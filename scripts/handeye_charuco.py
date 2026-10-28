#!/usr/bin/env python3
"""Manual eye-in-hand ChArUco calibration; passive SocketCAN + RealSense color.

All transforms T_A_B map coordinates from B to A. Lengths are metres.
No SDK connection or CAN transmit path is used.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import select
import socket
import struct
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
JOINT_IDS = (0x2A5, 0x2A6, 0x2A7, 0x2A9)


def write_json(path, data):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')
    tmp.replace(path)


def transform(rotation=None, translation=None):
    t = np.eye(4)
    if rotation is not None:
        t[:3, :3] = rotation
    if translation is not None:
        t[:3, 3] = np.asarray(translation).reshape(3)
    return t


def rpy_matrix(rpy):
    r, p, y = rpy
    rx = cv2.Rodrigues(np.array([r, 0., 0.]))[0]
    ry = cv2.Rodrigues(np.array([0., p, 0.]))[0]
    rz = cv2.Rodrigues(np.array([0., 0., y]))[0]
    return rz @ ry @ rx


class FK:
    """URDF chain, using the same joint1..7 angles as official NERO feedback."""
    def __init__(self, path, tip='gripper_flange', base='base_link'):
        tree = ET.parse(path).getroot()
        joints = {j.find('child').get('link'): j for j in tree.findall('joint')}
        self.chain = []
        link = tip
        while link in joints and link != base:
            j = joints[link]
            self.chain.insert(0, j)
            link = j.find('parent').get('link')
        self.base = link
        if link != base:
            raise ValueError(f'Base {base} is not an ancestor of {tip}')
        self.tip = tip
        moving = [j.get('name') for j in self.chain if j.get('type') != 'fixed']
        if set(moving) != {f'joint{i}' for i in range(1, 8)}:
            raise ValueError(f'Expected joint1..7 chain, got {moving}; check --tip/--urdf')

    def __call__(self, q):
        t = np.eye(4)
        for j in self.chain:
            o = j.find('origin')
            xyz = np.fromstring(o.get('xyz', '0 0 0'), sep=' ') if o is not None else np.zeros(3)
            rpy = np.fromstring(o.get('rpy', '0 0 0'), sep=' ') if o is not None else np.zeros(3)
            t = t @ transform(rpy_matrix(rpy), xyz)
            if j.get('type') != 'fixed':
                axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                axis /= np.linalg.norm(axis)
                angle = q[int(j.get('name').removeprefix('joint')) - 1]
                t = t @ transform(cv2.Rodrigues(axis * angle)[0])
        return t


def decode_joints(frames):
    # Vendor protocol: big-endian signed int32, 0.001 degree, seven axes.
    raw = []
    for cid in JOINT_IDS:
        values = struct.unpack('>ii', frames[cid][1])
        raw.extend(values[:1] if cid == 0x2A9 else values)
    return np.deg2rad(np.array(raw) / 1000.)


class PassiveJoints:
    def __init__(self, channel):
        self.bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Linux gives CAN_ERR_FLAG special filter semantics; exclude it from mask.
        mask = socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_SFF_MASK
        self.bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                            b''.join(struct.pack('=II', i, mask) for i in JOINT_IDS))
        self.bus.bind((channel,))
        self.lock = threading.Lock()
        self.history = deque(maxlen=600)
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        frames = {}
        updated = set()
        try:
            while not self.stop.is_set():
                if not select.select([self.bus], [], [], .05)[0]:
                    continue
                cid, size, data = struct.unpack('=IB3x8s', self.bus.recv(16))
                if cid not in JOINT_IDS or size != 8:
                    continue
                frames[cid] = (time.monotonic(), data)
                updated.add(cid)
                if len(updated) == 4:
                    times = [frames[i][0] for i in JOINT_IDS]
                    if max(times) - min(times) < .03:
                        with self.lock:
                            self.history.append((min(times), max(times), decode_joints(frames)))
                    updated.clear()
        except Exception as exc:
            self.error = str(exc)

    def snapshot(self, frame_time):
        if self.error:
            raise RuntimeError('CAN reader: ' + self.error)
        with self.lock:
            history = list(self.history)
        if not history or frame_time - history[-1][0] > .1:
            raise ValueError('七轴反馈缺失或超过 100ms，未保存')
        recent = [h for h in history if frame_time - .6 <= h[1] <= frame_time + .01]
        if len(recent) < 5 or recent[-1][1] - recent[0][0] < .45:
            raise ValueError('反馈不足，请停稳至少 0.6 秒再拍')
        if np.max(np.ptp(np.array([h[2] for h in recent]), axis=0)) > np.deg2rad(.3):
            raise ValueError('最近 0.6 秒关节变化超过 0.3 度，请停稳再拍')
        sample = min(recent, key=lambda h: abs(h[1] - frame_time))
        return sample[2], dict(frame_host_monotonic=frame_time,
                               joint_oldest_monotonic=sample[0], joint_newest_monotonic=sample[1],
                               pairing='host receipt; stationary window, not hardware synchronized')

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.bus.close()


class Board:
    def __init__(self, config):
        self.config = config
        if not 0 < config['marker_m'] < config['square_m']:
            raise ValueError('marker_m must be positive and smaller than square_m')
        d = getattr(cv2.aruco, config['dictionary'])
        self.dictionary = cv2.aruco.getPredefinedDictionary(d)
        args = (config['squares_x'], config['squares_y'], config['square_m'], config['marker_m'])
        if hasattr(cv2.aruco, 'CharucoBoard_create'):
            self.board = cv2.aruco.CharucoBoard_create(*args, self.dictionary)
            self.points = self.board.chessboardCorners
        else:
            self.board = cv2.aruco.CharucoBoard(args[:2], *args[2:], self.dictionary)
            self.points = self.board.getChessboardCorners()

    def detect(self, image, k=None, dist=None):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        preview = image.copy()
        if ids is None:
            return None, preview
        cv2.aruco.drawDetectedMarkers(preview, corners, ids)
        n, cc, ci = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board, cameraMatrix=k, distCoeffs=dist)
        if ci is None or n < 6:
            return None, preview
        cv2.aruco.drawDetectedCornersCharuco(preview, cc, ci)
        obj = self.points[ci.flatten()]
        if np.linalg.matrix_rank(obj[:, :2] - obj[:, :2].mean(axis=0)) < 2:
            return None, preview
        result = dict(corner_ids=ci.flatten().tolist(), corners_px=cc.reshape(-1, 2).tolist(),
                      marker_ids=ids.flatten().tolist())
        if k is not None:
            ok, r, t = cv2.solvePnP(obj, cc, k, dist)
            if not ok or t[2, 0] <= 0:
                return None, preview
            projected = cv2.projectPoints(obj, r, t, k, dist)[0]
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - cc.reshape(-1, 2))**2, axis=1))))
            result.update(T_camera_board=transform(cv2.Rodrigues(r)[0], t).tolist(), reprojection_rms_px=error)
            cv2.drawFrameAxes(preview, k, dist, r, t, .06)
        return result, preview


def solve(samples):
    if len(samples) < 6:
        raise ValueError('至少采集 6 组；建议 15～25 组不同朝向的数据')
    g = [np.array(s['T_base_gripper'], dtype=float) for s in samples]
    c = [np.array(s['T_camera_board'], dtype=float) for s in samples]
    vectors = [cv2.Rodrigues(a[:3, :3].T @ b[:3, :3])[0].flatten()
               for i, a in enumerate(g) for b in g[i+1:]]
    singular = np.linalg.svd(np.array(vectors), compute_uv=False)
    if singular[0] < .15 or singular[1] < .05 or singular[1] / singular[0] < .05:
        raise ValueError('姿态退化：请绕至少两个不同方向明显转动末端后重新采集')
    r, t = cv2.calibrateHandEye([a[:3, :3] for a in g], [a[:3, 3] for a in g],
                               [a[:3, :3] for a in c], [a[:3, 3] for a in c],
                               method=cv2.CALIB_HAND_EYE_PARK)
    x = transform(r, t)
    if not np.isfinite(x).all() or abs(np.linalg.det(r) - 1) > 1e-4:
        raise ValueError('求解失败或结果无效')
    fixed = [a @ x @ b for a, b in zip(g, c)]
    center = np.mean([a[:3, 3] for a in fixed], axis=0)
    u, _, vt = np.linalg.svd(np.mean([a[:3, :3] for a in fixed], axis=0))
    mean_r = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    spread = [np.linalg.norm(a[:3, 3] - center) * 1000 for a in fixed]
    angles = [np.linalg.norm(cv2.Rodrigues(fixed[0][:3, :3].T @ a[:3, :3])[0]) * 180 / np.pi for a in fixed]
    return dict(T_gripper_camera=x.tolist(), T_camera_gripper=np.linalg.inv(x).tolist(),
                T_base_board_reference=transform(mean_r, center).tolist(),
                transform_convention='T_A_B maps coordinates from B to A; metres; camera=color optical frame',
                method='OpenCV PARK', samples=len(samples), rotation_excitation_singular_values=singular.tolist(),
                training_board_position_rms_mm=float(np.sqrt(np.mean(np.square(spread)))),
                training_board_position_max_mm=float(max(spread)),
                training_board_orientation_max_from_first_deg=float(max(angles)),
                validation='Training consistency only; independently verify with new poses before use.')


def camera_geometry(intr):
    """Use SDK inverse-Brown deprojection to rectify; never reinterpret its coefficients."""
    import pyrealsense2 as rs
    k = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.]])
    if intr.model == rs.distortion.inverse_brown_conrady:
        from realsense_rectify import build_maps
        return k, np.zeros(5), build_maps(intr)
    if intr.model in (rs.distortion.none, rs.distortion.brown_conrady):
        return k, np.array(intr.coeffs) if intr.model != rs.distortion.none else np.zeros(5), None
    raise ValueError(f'Unsupported color distortion {intr.model}')


def select_color_profile(profiles):
    """Prioritize spatial resolution for stationary corner detection, then FPS."""
    candidates = [p for p in profiles if 5 <= p[2] <= 30]
    if not candidates:
        raise ValueError('相机没有可用的 5～30fps RGB 配置')
    return max(candidates, key=lambda p: (p[0]*p[1], p[2]))


def camera_config(serial=None):
    import pyrealsense2 as rs
    context = rs.context()
    devices = list(context.query_devices())
    if not serial and len(devices) > 1:
        raise ValueError('多个 RealSense，请用 --serial 指定末端相机')
    device = next((d for d in devices if not serial or d.get_info(rs.camera_info.serial_number) == serial), None)
    if device is None:
        raise ValueError('未检测到指定 RealSense 相机')
    cfg = rs.config()
    cfg.enable_device(device.get_info(rs.camera_info.serial_number))
    profiles = [p for sensor in device.query_sensors() for p in sensor.get_stream_profiles() if p.is_video_stream_profile()]
    usb = device.get_info(rs.camera_info.usb_type_descriptor) if device.supports(rs.camera_info.usb_type_descriptor) else ''
    color = {(p.as_video_stream_profile().width(), p.as_video_stream_profile().height(), p.fps())
             for p in profiles if p.stream_type() == rs.stream.color and p.format() == rs.format.rgb8}
    width, height, fps = select_color_profile(color)
    # ChArUco PnP uses RGB and intrinsics; a depth stream is unnecessary.
    cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    print(f'RealSense USB {usb or "unknown"}, RGB-only {width}x{height}@{fps}', flush=True)
    return cfg


def color_bgr(frame):
    import pyrealsense2 as rs
    image = np.asanyarray(frame.get_data())
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if frame.profile.format() == rs.format.rgb8 else image.copy()


def validate(reference_session, new_session):
    reference_session, new_session = Path(reference_session), Path(new_session)
    meta = json.loads((reference_session / 'session.json').read_text())
    new_meta = json.loads((new_session / 'session.json').read_text())
    for key in ('camera_serial', 'board', 'K', 'distortion', 'can', 'urdf_sha256', 'base_frame', 'gripper_frame'):
        if meta[key] != new_meta[key]:
            raise ValueError(f'Validation session differs in {key}')
    result = json.loads((reference_session / 'result.json').read_text())
    x = np.array(result['T_gripper_camera'])
    reference = np.array(result['T_base_board_reference'])
    samples = json.loads((new_session / 'samples.json').read_text())
    if not samples:
        raise ValueError('Validation session has no samples')
    rows = []
    for sample in samples:
        estimated = np.array(sample['T_base_gripper']) @ x @ np.array(sample['T_camera_board'])
        rows.append(dict(image=sample['image'],
                         position_error_mm=float(np.linalg.norm(estimated[:3, 3] - reference[:3, 3]) * 1000),
                         orientation_error_deg=float(np.linalg.norm(cv2.Rodrigues(reference[:3, :3].T @ estimated[:3, :3])[0]) * 180 / np.pi)))
    report = dict(reference_session=str(reference_session.resolve()), samples=rows,
                  position_rms_mm=float(np.sqrt(np.mean([r['position_error_mm']**2 for r in rows]))),
                  position_max_mm=max(r['position_error_mm'] for r in rows),
                  orientation_max_deg=max(r['orientation_error_deg'] for r in rows),
                  condition='Board and arm base must remain fixed between sessions. No automatic accuracy acceptance.')
    write_json(new_session / 'validation.json', report)
    return report


def collect(args, config):
    import pyrealsense2 as rs
    config = dict(config, marker_m=args.marker_mm / 1000., marker_size_verified=True)
    board = Board(config)
    fk = FK(args.urdf, args.tip)
    out = Path(args.session) if args.session else ROOT / 'calibration_data' / time.strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    (out / 'images').mkdir()
    pipe = rs.pipeline()
    rsconfig = camera_config(args.serial)
    bus = None
    started = False
    try:
        profile = pipe.start(rsconfig)
        started = True
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        k, dist, maps = camera_geometry(intr)
        metadata = dict(board=config, K=k.tolist(), distortion=dist.tolist(),
                        distortion_model='plumb_bob', original_distortion_model=str(intr.model),
                        original_distortion=list(intr.coeffs), rectified=maps is not None,
                        resolution=[intr.width, intr.height],
                        camera_serial=profile.get_device().get_info(rs.camera_info.serial_number),
                        can=args.can, urdf=str(Path(args.urdf).resolve()),
                        urdf_sha256=hashlib.sha256(Path(args.urdf).read_bytes()).hexdigest(),
                        base_frame=fk.base, gripper_frame=fk.tip, mode='eye-in-hand', units='m',
                        opencv_version=cv2.__version__)
        write_json(out / 'session.json', metadata)
        samples = []
        write_json(out / 'samples.json', samples)
        bus = PassiveJoints(args.can)
        print(f'采集目录: {out}\nSPACE/c 保存；s 求解；q 退出。固定板，移动末端相机，停稳后拍。', flush=True)
        for _ in range(30):
            pipe.wait_for_frames(timeout_ms=2000)
        while True:
            frame = pipe.wait_for_frames(timeout_ms=2000).get_color_frame()
            stamp = time.monotonic()
            raw = color_bgr(frame)
            image = cv2.remap(raw, *maps, interpolation=cv2.INTER_LINEAR) if maps is not None else raw
            detection, preview = board.detect(image, k, dist)
            text = f'samples={len(samples)} corners={len(detection["corner_ids"]) if detection else 0} SPACE:capture s:solve q:quit'
            cv2.putText(preview, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2)
            cv2.imshow('NERO ChArUco eye-in-hand', preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            if key == ord('s'):
                try:
                    result = solve(samples)
                    result.update(base_frame=fk.base, gripper_frame=fk.tip)
                    write_json(out / 'result.json', result)
                    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
                except ValueError as exc:
                    print(exc, flush=True)
            if key not in (32, ord('c')):
                continue
            try:
                if detection is None or detection['reprojection_rms_px'] > args.max_reprojection:
                    raise ValueError('角点不足或重投影误差过大，未保存')
                q, timing = bus.snapshot(stamp)
                # Also require stillness immediately after the selected image.
                time.sleep(.15)
                after, _ = bus.snapshot(time.monotonic())
                if np.max(np.abs(after - q)) > np.deg2rad(.3):
                    raise ValueError('拍摄后检测到运动，未保存')
                index = len(samples)
                image_name = f'images/{index:04d}.png'
                if not cv2.imwrite(str(out / image_name), image):
                    raise OSError('原图保存失败')
                raw_name = f'images/{index:04d}_raw.png'
                if maps is not None and not cv2.imwrite(str(out / raw_name), raw):
                    raise OSError('未校正原图保存失败')
                sample = dict(detection, image=image_name, q_rad=q.tolist(),
                              raw_image=raw_name if maps is not None else image_name,
                              T_base_gripper=fk(q).tolist(), timing=timing,
                              camera_timestamp_ms=frame.get_timestamp(),
                              camera_timestamp_domain=str(frame.get_frame_timestamp_domain()),
                              camera_frame_number=frame.get_frame_number())
                samples.append(sample)
                write_json(out / 'samples.json', samples)
                print(f'已保存 #{index+1}, corners={len(sample["corner_ids"])}, RMS={sample["reprojection_rms_px"]:.3f}px', flush=True)
            except ValueError as exc:
                print(exc, flush=True)
    finally:
        if bus is not None:
            bus.close()
        if started:
            pipe.stop()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/handeye_charuco.json')
    sub = parser.add_subparsers(dest='command', required=True)
    cap = sub.add_parser('collect', help='RealSense RGB + passive seven-joint feedback')
    cap.add_argument('--can', required=True, help='Explicit SocketCAN channel, no assumed arm mapping')
    cap.add_argument('--marker-mm', required=True, type=float, help='Measured marker black outer edge in mm')
    cap.add_argument('--serial', help='RealSense serial (required if multiple cameras are attached)')
    cap.add_argument('--session', help='New output directory, must not exist')
    cap.add_argument('--urdf', type=Path, default=ROOT / 'config/gravity/nero_standard_gripper_no_camera.urdf')
    cap.add_argument('--tip', default='gripper_flange')
    cap.add_argument('--max-reprojection', type=float, default=1.5)
    offline = sub.add_parser('solve', help='Offline solve saved session; no camera/CAN opened')
    offline.add_argument('session', type=Path)
    check = sub.add_parser('validate', help='Evaluate held-out poses without recalibrating')
    check.add_argument('reference_session', type=Path)
    check.add_argument('new_session', type=Path)
    inspect = sub.add_parser('inspect', help='Detect board in saved image; no camera/CAN opened')
    inspect.add_argument('image', type=Path)
    inspect.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'validate':
        print(json.dumps(validate(args.reference_session, args.new_session), ensure_ascii=False, indent=2))
        return
    if args.command == 'solve':
        samples = json.loads((args.session / 'samples.json').read_text())
        meta = json.loads((args.session / 'session.json').read_text())
        result = solve(samples)
        result.update(base_frame=meta['base_frame'], gripper_frame=meta['gripper_frame'])
        write_json(args.session / 'result.json', result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return
    config = json.loads(args.config.read_text())
    if args.command == 'inspect':
        raw = cv2.imread(str(args.image))
        if raw is None:
            raise ValueError(f'Cannot read {args.image}')
        result, preview = Board(config).detect(raw)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), preview):
            raise OSError('Preview save failed')
        print(json.dumps(result, indent=2))
        if result is None:
            raise ValueError('Board not detected')
    else:
        collect(args, config)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n采集已退出，已保存数据保留。')
