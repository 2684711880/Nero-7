#!/usr/bin/env python3
"""Preview a ChArUco corner pointing target from existing RViz inputs; no motion."""
import argparse
from array import array
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from handeye_charuco import ROOT, Board, FK, PassiveJoints, transform, write_json


def pointing_target(base_gripper, gripper_camera, camera_board, corner_board, standoff, tcp_offset):
    camera_corner = (camera_board @ np.r_[corner_board, 1.])[:3]
    normal = camera_board[:3,2].copy()
    if normal @ camera_board[:3,3] > 0:
        normal *= -1
    base_camera = base_gripper @ gripper_camera
    corner_base = (base_camera @ np.r_[camera_corner, 1.])[:3]
    tcp_target = corner_base + standoff * base_camera[:3,:3] @ normal
    flange_target = tcp_target - base_gripper[:3,:3] @ np.asarray(tcp_offset)
    return dict(corner_base_m=corner_base.tolist(), target_tcp_base_m=tcp_target.tolist(),
                target_flange_base_m=flange_target.tolist(), orientation='keep_current',
                R_base_flange=base_gripper[:3,:3].tolist())


def select_corner(board, target, marker_id=None):
    if target == 'board':
        cfg = board.config
        # This user's printed board is portrait (5 columns, 7 rows), a quarter
        # turn from OpenCV's canonical 7x5 landscape layout. On the actual
        # print, +X points down and +Y right: the portrait top-left is (0,0).
        points = board.points
        if points[0,1] > points[-1,1]:
            raise ValueError('Different OpenCV ChArUco convention; verify board coordinates first')
        return np.array([0., 0., 0.])
    if marker_id is None:
        raise ValueError('--target marker requires --marker-id')
    ids = board.board.ids.flatten() if hasattr(board.board, 'ids') else board.board.getIds().flatten()
    obj = board.board.objPoints if hasattr(board.board, 'objPoints') else board.board.getObjPoints()
    found = np.flatnonzero(ids == marker_id)
    if len(found) != 1:
        raise ValueError('Marker ID not on this board')
    return obj[int(found[0])][0].astype(float)


class Preview(Node):
    def __init__(self):
        super().__init__('nero_corner_preview')
        self.image = self.info = None
        self.create_subscription(Image, '/handeye/color/image_raw', self.on_image, 5)
        self.create_subscription(CameraInfo, '/handeye/color/camera_info', self.on_info, 5)

    def on_image(self, msg):
        self.image = msg

    def on_info(self, msg):
        self.info = msg


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--can', required=True)
    p.add_argument('--calibration', type=Path, default=ROOT/'config/handeye/can1_d405.json')
    p.add_argument('--target', choices=['board','marker'], required=True)
    p.add_argument('--marker-id', type=int)
    p.add_argument('--tcp-offset-mm', type=float, nargs=3, required=True,
                   help='Physical pointing tip XYZ relative to gripper_flange, in mm')
    p.add_argument('--standoff', type=float, default=.10, help='Metres outward from printed face')
    a = p.parse_args()
    if not np.isfinite(a.tcp_offset_mm).all() or not .03 <= a.standoff <= .5:
        p.error('Finite TCP offset and standoff 0.03..0.5m required')
    meta = json.loads(a.calibration.read_text())
    if meta['can'] != a.can or meta['gripper_frame'] != 'gripper_flange':
        raise ValueError('Calibration arm/frame mismatch')
    cfg = json.loads((ROOT/'config/handeye_charuco.json').read_text())
    board = Board(cfg)
    corner = select_corner(board,a.target,a.marker_id)
    fk = FK(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf')
    out = ROOT/'calibration_data'/('point_preview_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    node = Preview()
    bus = None
    try:
        bus = PassiveJoints(a.can)
        deadline = time.monotonic()+8
        result = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.image is None or node.info is None:
                continue
            im, info = node.image, node.info
            stamp = im.header.stamp.sec + im.header.stamp.nanosec*1e-9
            if not 0 <= time.time()-stamp < .25 or im.encoding != 'bgr8':
                continue
            if im.header.frame_id != meta['camera_frame']:
                raise ValueError('RGB camera frame mismatch')
            try:
                q, timing = bus.snapshot(time.monotonic())
            except ValueError:
                continue
            image = np.frombuffer(im.data,dtype=np.uint8).reshape(im.height,im.step)[:,:im.width*3].reshape(im.height,im.width,3).copy()
            k, d = np.array(info.k).reshape(3,3), np.array(info.d)
            det, preview = board.detect(image,k,d)
            if det is None:
                continue
            if det['reprojection_rms_px'] > 1.5:
                continue
            camera_board = np.array(det['T_camera_board'])
            result = pointing_target(fk(q),np.array(meta['T_gripper_camera']),camera_board,corner,
                                     a.standoff,np.array(a.tcp_offset_mm)/1000.)
            r = cv2.Rodrigues(camera_board[:3,:3])[0]
            uv = cv2.projectPoints(corner.reshape(1,3),r,camera_board[:3,3],k,d)[0].reshape(2)
            cv2.circle(preview,tuple(uv.astype(int)),10,(0,0,255),3)
            cv2.putText(preview,'TARGET: red circle; PREVIEW ONLY',(10,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
            cv2.imwrite(str(out/'target.png'),preview)
            cv2.imwrite(str(out/'color.png'),image)
            result.update(calibration=str(a.calibration),calibration_status=meta['status'],can=a.can,
                          target=a.target,marker_id=a.marker_id,corner_board_m=corner.tolist(),
                          q_rad=q.tolist(),tcp_offset_mm=a.tcp_offset_mm,standoff_m=a.standoff,
                          reprojection_rms_px=det['reprojection_rms_px'],motion_sent=False,
                          note='Board top-left follows printed board orientation; inspect red circle. No IK or trajectory checked.')
            write_json(out/'target.json',result)
            print(json.dumps(result,ensure_ascii=False,indent=2));print('输出目录:',out)
            break
        if result is None:
            raise RuntimeError('No fresh stationary board observation. Keep run_handeye.sh active and board visible.')
    finally:
        if bus is not None:bus.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
