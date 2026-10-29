#!/usr/bin/env python3
"""Gemini 336L native-depth hole preview. Never connects to a robot."""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from pyorbbecsdk import Context, Pipeline, OBCameraDistortionModel, OBFormat
from depth_hole import detect_depth_hole


def color_bgr(frame):
    data = np.frombuffer(frame.get_data(), np.uint8)
    w, h, fmt = frame.get_width(), frame.get_height(), frame.get_format()
    if fmt == OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('Invalid MJPEG frame')
        return image
    if fmt in (OBFormat.RGB, OBFormat.BGR):
        image = data.reshape(h, w, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if fmt == OBFormat.RGB else image.copy()
    if fmt in (OBFormat.YUYV, OBFormat.YUY2, OBFormat.UYVY):
        code = cv2.COLOR_YUV2BGR_UYVY if fmt == OBFormat.UYVY else cv2.COLOR_YUV2BGR_YUY2
        return cv2.cvtColor(data.reshape(h, w, 2), code)
    raise ValueError(f'Unsupported color format: {fmt}')


def depth_geometry(profile):
    intr, dist = profile.get_intrinsic(), profile.get_distortion()
    k = np.array([[intr.fx, 0., intr.cx], [0., intr.fy, intr.cy], [0., 0., 1.]])
    if dist.model == OBCameraDistortionModel.NONE:
        coeffs = np.zeros(8)
    elif dist.model in (OBCameraDistortionModel.BROWN_CONRADY, OBCameraDistortionModel.BROWN_CONRADY_K6):
        coeffs = np.array([dist.k1, dist.k2, dist.p1, dist.p2, dist.k3, dist.k4, dist.k5, dist.k6])
    else:
        raise ValueError(f'Unsupported depth distortion model: {dist.model}')
    maps = cv2.initUndistortRectifyMap(k, coeffs, np.eye(3), k,
                                        (intr.width, intr.height), cv2.CV_32FC1)
    return k, maps, (intr.width, intr.height)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--serial', help='Required when multiple Orbbec cameras are present')
    ap.add_argument('--min-diameter-mm', type=float, default=42.5)
    ap.add_argument('--max-diameter-mm', type=float, default=120.)
    args = ap.parse_args()
    if not 0 < args.min_diameter_mm < args.max_diameter_mm:
        ap.error('Invalid diameter range')
    ctx = Context()
    devices = ctx.query_devices()
    if not devices.get_count():
        raise RuntimeError('未发现奥比中光相机，请检查连接、供电和 USB 权限')
    if not args.serial and devices.get_count() != 1:
        raise RuntimeError('检测到多台奥比中光相机，请用 --serial 指定')
    device = devices.get_device_by_serial_number(args.serial) if args.serial else devices.get_device_by_index(0)
    if device is None:
        raise RuntimeError('找不到指定序列号的相机')
    info = device.get_device_info()
    if '336' not in info.get_name():
        raise RuntimeError(f'需要 Gemini 336L，当前设备：{info.get_name()}')
    pipe = Pipeline(device)
    started = False
    name = '336L depth hole | q quit | s save'
    color_name = '336L color (not aligned)'
    try:
        pipe.start()
        started = True
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(color_name, cv2.WINDOW_NORMAL)
        print(f'{info.get_name()} {info.get_serial_number()} | q/Esc: quit; s: save | no arm connection', flush=True)
        geometry = None
        last_frame = time.monotonic()
        while True:
            frames = pipe.wait_for_frames(1000)
            d = frames.get_depth_frame() if frames else None
            c = frames.get_color_frame() if frames else None
            if d is None or c is None:
                if time.monotonic() - last_frame > 15:
                    raise RuntimeError('15 秒未收到完整彩色和深度帧')
                continue
            last_frame = time.monotonic()
            if geometry is None:
                geometry = depth_geometry(d.get_stream_profile().as_video_stream_profile())
            k, maps, size = geometry
            raw = np.frombuffer(d.get_data(), np.uint16).reshape(d.get_height(), d.get_width()).copy()
            if raw.shape[::-1] != size:
                raise ValueError('Depth frame size differs from its intrinsics')
            rect = cv2.remap(raw, *maps, interpolation=cv2.INTER_NEAREST)
            # Resizing keeps the detector responsive; intrinsics follow the image scaling.
            width = min(640, size[0])
            height = round(size[1] * width / size[0])
            depth = cv2.resize(rect, (width, height), interpolation=cv2.INTER_NEAREST).astype(np.float32) * d.get_depth_scale() / 1000.
            scaled_k = k.copy()
            scaled_k[0, :] *= width / size[0]
            scaled_k[1, :] *= height / size[1]
            heat = cv2.applyColorMap(np.uint8(np.clip(depth / 3., 0, 1) * 255), cv2.COLORMAP_TURBO)
            heat[depth <= 0] = 0
            result = None
            try:
                result = detect_depth_hole(depth, scaled_k, (args.min_diameter_mm/1000, args.max_diameter_mm/1000))
                cv2.ellipse(heat, result['ellipse'], (0, 255, 0), 2)
                p = result['center']
                label = f'Hole: {result["radius"]*2000:.1f} mm | XYZ {p[0]*1000:.0f}, {p[1]*1000:.0f}, {p[2]*1000:.0f} mm'
                color = (0, 255, 0)
            except ValueError as exc:
                label, color = str(exc), (0, 0, 255)
            cv2.putText(heat, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1, cv2.LINE_AA)
            cv2.putText(heat, 'Depth optical frame | black: invalid | q: quit | s: save', (8, 46), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
            rgb = color_bgr(c)
            cv2.imshow(name, heat)
            cv2.imshow(color_name, rgb)
            key = cv2.waitKey(1) & 255
            if key == ord('s'):
                folder = Path(__file__).resolve().parents[1] / 'captures' / time.strftime('orbbec_hole_%Y%m%d_%H%M%S')
                folder.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(folder / 'color.png'), rgb)
                cv2.imwrite(str(folder / 'depth_raw.png'), raw)
                cv2.imwrite(str(folder / 'detection.png'), heat)
                metadata = dict(camera=info.get_name(), serial=info.get_serial_number(), depth_scale_mm=d.get_depth_scale(), depth_intrinsics=k.tolist(), detection_intrinsics=scaled_k.tolist(), depth_frame='native depth optical', detection=None if result is None else dict(center_m=result['center'].tolist(), normal=result['normal'].tolist(), diameter_m=2*result['radius'], plane_rms_m=result['plane_rms'], circle_rms_m=result['circle_rms']))
                (folder / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
                print(f'Saved {folder}', flush=True)
            if key in (ord('q'), 27):
                break
    finally:
        if started:
            pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        raise SystemExit(f'相机预览失败: {exc}')
