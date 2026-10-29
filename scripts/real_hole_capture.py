#!/usr/bin/env python3
"""Capture one NERO right D405 hole observation with passive joint feedback."""
import argparse
from collections import deque
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from handeye_charuco import FK
from pointing_model import mount_matrix
from real_hole_geometry import transform_depth_observation

ROOT = Path(__file__).resolve().parents[1]
from depth_hole import detect_depth_hole


class Feedback(Node):
    def __init__(self):
        super().__init__('nero_real_hole_feedback')
        self.samples = {side: deque(maxlen=100) for side in ('left', 'right')}
        self.grippers = {}
        self.sub_handles = [self.create_subscription(JointState, f'/{s}/feedback/joint_states', lambda m, side=s: self.receive(side, m), 1) for s in self.samples]

    def receive(self, side, msg):
        values = dict(zip(msg.name, msg.position))
        names = [f'joint{i}' for i in range(1, 8)]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not 0 <= time.time() - stamp < .3:
            return
        if 'gripper' in values and np.isfinite(values['gripper']):
            self.grippers[side] = (time.monotonic(), float(values['gripper']))
        if all(n in values and np.isfinite(values[n]) for n in names):
            self.samples[side].append((time.monotonic(), np.array([values[n] for n in names])))

    def stationary(self):
        now = time.monotonic()
        result = {}
        for side, samples in self.samples.items():
            snapshot = list(samples)
            recent = [q for t, q in snapshot if now - t < .6]
            if len(recent) < 5 or now - snapshot[-1][0] > .2 or np.max(np.ptp(recent, axis=0)) > np.deg2rad(.3):
                raise RuntimeError(f'{side} arm feedback missing, stale, or moving')
            result[side] = recent[-1].copy()
        return result

    def gripper_width(self, side):
        sample = self.grippers.get(side)
        if sample is None or time.monotonic() - sample[0] > .3:
            raise RuntimeError(f'{side} gripper feedback missing or stale')
        return sample[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default='262422270020')
    parser.add_argument('--output', type=Path, default=ROOT/'hardware_ws/validation/real_hole/observation.json')
    args = parser.parse_args()
    metadata = json.loads((ROOT/'config/handeye/right/d405.json').read_text())
    if args.serial != metadata['camera_serial']:
        raise ValueError('Camera serial differs from right hand-eye calibration')
    rclpy.init()
    node = Feedback()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    pipe = rs.pipeline()
    started = False
    try:
        cfg = rs.config()
        cfg.enable_device(args.serial)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        profile = pipe.start(cfg)
        started = True
        depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = depth_stream.intrinsics
        if any(abs(x) > 1e-8 for x in intr.coeffs):
            raise ValueError('Depth intrinsics have nonzero distortion')
        k = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.]])
        extr = depth_stream.get_extrinsics_to(color_stream)
        color_depth = np.eye(4)
        color_depth[:3, :3] = np.asarray(extr.rotation).reshape(3, 3, order='F')
        color_depth[:3, 3] = extr.translation
        for _ in range(30):
            pipe.wait_for_frames(5000)
        before = node.stationary()
        frame = pipe.wait_for_frames(5000).get_depth_frame()
        stamp = time.time()
        depth = np.asanyarray(frame.get_data()).astype(np.float32) * frame.get_units()
        hole = detect_depth_hole(depth, k)
        after = node.stationary()
        if any(np.max(np.abs(before[s]-after[s])) > np.deg2rad(.3) for s in before):
            raise RuntimeError('Arm moved while capturing/detecting')
        fk = FK(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf')
        base_flange = mount_matrix('right') @ fk(before['right'])
        center, normal = transform_depth_observation(base_flange, metadata['T_gripper_camera'], color_depth, hole['center'], hole['normal'])
        report = dict(source='right_d405_native_depth', serial=args.serial, observed_wall_time=stamp,
                      q_left_rad=before['left'].tolist(), q_right_rad=before['right'].tolist(),
                      center_depth_m=hole['center'].tolist(), normal_depth=hole['normal'].tolist(),
                      center_base_m=center.tolist(), normal_base=normal.tolist(), diameter_m=2*hole['radius'],
                      plane_rms_m=hole['plane_rms'], circle_rms_m=hole['circle_rms'],
                      T_color_depth=color_depth.tolist(), motion_sent=False)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        if started:
            pipe.stop()
        executor.shutdown()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
