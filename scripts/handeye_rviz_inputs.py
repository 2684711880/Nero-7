#!/usr/bin/env python3
"""Real NERO passive feedback/FK and rectified RealSense images for RViz."""
import argparse
from array import array as ByteArray
import json
from pathlib import Path
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from handeye_charuco import ROOT, FK, PassiveJoints, camera_geometry, camera_config, color_bgr, write_json


def quaternion(r):
    # Rodrigues axis-angle avoids numerical instability near rotations of pi.
    v = cv2.Rodrigues(r)[0].reshape(3)
    angle = np.linalg.norm(v)
    xyz = v * (np.sin(angle/2) / angle) if angle > 1e-12 else v/2
    return [*map(float, xyz), float(np.cos(angle/2))]


class Inputs(Node):
    def __init__(self, args):
        super().__init__('nero_handeye_inputs')
        self.args = args
        self.pipe = None
        self.bus = None
        self.fk = FK(args.urdf)
        self.tf = TransformBroadcaster(self)
        self.image_pub = self.create_publisher(Image, '/handeye/color/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/handeye/color/camera_info', 10)
        self.images = self.joints = 0
        self.last_joint = None
        self.last_image = None
        self.last_capture = time.monotonic()
        self.last_restart = float('-inf')
        self.camera_serial = args.serial
        try:
            self.start_camera()
            if not getattr(args, 'camera_only', False):
                self.bus = PassiveJoints(args.can)
            self.create_timer(.03, self.on_frame)
            if not getattr(args,'no_robot_tf',False) and not getattr(args, 'camera_only', False):
                self.create_timer(.02, self.on_joint)
            self.get_logger().info(f'Ready: {args.can}, {self.intr.model}, rectified={self.maps is not None}. No CAN writes.')
        except BaseException:
            self.close()
            raise

    def start_camera(self):
        cfg = camera_config(self.camera_serial)
        pipe = rs.pipeline()
        try:
            profile = pipe.start(cfg)
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            k, dist, maps = camera_geometry(intr)
        except BaseException:
            try: pipe.stop()
            except RuntimeError: pass
            raise
        self.pipe = pipe
        device = profile.get_device()
        self.camera_serial = device.get_info(rs.camera_info.serial_number)
        camera_name = device.get_info(rs.camera_info.name)
        self.intr, self.k, self.dist, self.maps = intr, k, dist, maps
        self.last_capture = time.monotonic()
        write_json(Path(self.args.session) / 'inputs.json', dict(
            can=self.args.can, serial=self.camera_serial, camera_name=camera_name,
            fps=profile.get_stream(rs.stream.color).fps(),
            raw_distortion=str(intr.model), raw_coefficients=list(intr.coeffs),
            resolution=[intr.width, intr.height], K=k.tolist(), D=dist.tolist(),
            rectified=maps is not None, base_frame=self.fk.base, gripper_frame=self.fk.tip,
            mode='camera-only' if getattr(self.args, 'camera_only', False) else 'eye-in-hand',
            no_motor_commands=True))

    def recover_camera(self, now):
        # A USB re-enumeration leaves the old SDK pipeline alive but permanently empty.
        if now - self.last_capture < 3 or now - self.last_restart < 5:
            return
        self.last_restart = now
        self.get_logger().warning('RealSense: no color frames for 3s; reopening the same camera serial.')
        if self.pipe is not None:
            try: self.pipe.stop()
            except RuntimeError: pass
            self.pipe = None
        try:
            self.start_camera()
        except (RuntimeError, ValueError) as exc:
            self.get_logger().warning(f'RealSense reconnect pending: {exc}')

    def on_joint(self):
        now = time.monotonic()
        try:
            q, timing = self.bus.snapshot(now)
        except ValueError:
            return  # Do not refresh TF when feedback is stale or the arm is moving.
        age = now - timing['joint_oldest_monotonic']
        stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=max(0., age))).to_msg()
        t = self.fk(q)
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.fk.base
        msg.child_frame_id = self.fk.tip
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, t[:3, 3])
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = quaternion(t[:3, :3])
        self.tf.sendTransform(msg)
        self.joints += 1
        self.last_joint = now

    def on_frame(self):
        try:
            frames = self.pipe.poll_for_frames() if self.pipe is not None else None
        except RuntimeError as exc:
            self.get_logger().warning(f'RealSense capture interrupted: {exc}')
            frames = None
        if not frames:
            self.recover_camera(time.monotonic())
            return
        color = frames.get_color_frame()
        if not color:
            self.recover_camera(time.monotonic())
            return
        self.last_capture = time.monotonic()
        raw = color_bgr(color)
        image = cv2.remap(raw, *self.maps, interpolation=cv2.INTER_LINEAR) if self.maps is not None else raw
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'handeye_color_optical'
        msg.height, msg.width = image.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = ByteArray('B', image.tobytes())
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = msg.width, msg.height
        info.distortion_model = 'plumb_bob'
        info.d = self.dist.tolist()
        info.k = self.k.flatten().tolist()
        info.r = np.eye(3).flatten().tolist()
        info.p = np.c_[self.k, np.zeros(3)].flatten().tolist()
        self.info_pub.publish(info)
        self.image_pub.publish(msg)
        self.images += 1
        self.last_image = time.monotonic()
        if self.images == 10:
            write_json(self.args.session/'ready.json', dict(images=self.images))

    def summary(self):
        now = time.monotonic()
        return dict(images=self.images, joint_transforms=self.joints,
                    image_age_s=None if self.last_image is None else now-self.last_image,
                    joint_age_s=None if self.last_joint is None else now-self.last_joint,
                    no_motor_commands=True)

    def close(self):
        if self.bus is not None:
            self.bus.close()
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except RuntimeError:
                pass


def parse_cli(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can')
    parser.add_argument('--camera-only', action='store_true', help='Camera images only; no CAN socket or robot TF')
    parser.add_argument('--serial')
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, default=ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf')
    parser.add_argument('--check-seconds', type=float, default=0.)
    parser.add_argument('--no-robot-tf', action='store_true', help='Robot state publisher owns joint TF')
    # launch_ros adds --ros-args / remaps; they are not application arguments.
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=argv)[1:])
    if not args.camera_only and not args.can:
        parser.error('--can is required unless --camera-only is selected')
    return args


def main():
    args = parse_cli()
    args.session.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = None
    try:
        node = Inputs(args)
        deadline = time.monotonic() + args.check_seconds
        while rclpy.ok() and (args.check_seconds <= 0 or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            report = node.summary()
            write_json(args.session/'summary.json', report)
            print(json.dumps(report, indent=2))
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
