#!/usr/bin/env python3
"""Fixed Orbbec color camera + passive left-arm FK for RViz eye-to-hand calibration."""
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
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from pyorbbecsdk import (Context, Pipeline, Config, OBSensorType, OBFormat,
                        OBCameraDistortionModel)
from handeye_charuco import ROOT, FK, PassiveJoints, write_json


def color_bgr(frame):
    data = np.frombuffer(frame.get_data(), dtype=np.uint8)
    w, h, fmt = frame.get_width(), frame.get_height(), frame.get_format()
    if fmt == OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('Invalid MJPEG color frame')
        return image
    if fmt in (OBFormat.RGB, OBFormat.BGR):
        image = data.reshape(h, w, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if fmt == OBFormat.RGB else image.copy()
    if fmt in (OBFormat.YUYV, OBFormat.YUY2, OBFormat.UYVY):
        code = cv2.COLOR_YUV2BGR_UYVY if fmt == OBFormat.UYVY else cv2.COLOR_YUV2BGR_YUY2
        return cv2.cvtColor(data.reshape(h, w, 2), code)
    raise ValueError(f'Unsupported color format: {fmt}')


def camera_geometry(intr, distortion):
    k = np.array([[intr.fx, 0., intr.cx], [0., intr.fy, intr.cy], [0., 0., 1.]])
    if distortion.model == OBCameraDistortionModel.NONE:
        d = np.zeros(8)
    elif distortion.model in (OBCameraDistortionModel.BROWN_CONRADY,
                               OBCameraDistortionModel.BROWN_CONRADY_K6):
        d = np.array([distortion.k1, distortion.k2, distortion.p1, distortion.p2,
                      distortion.k3, distortion.k4, distortion.k5, distortion.k6])
    else:
        raise ValueError(f'Unsupported Orbbec distortion model: {distortion.model}; do not guess intrinsics')
    maps = cv2.initUndistortRectifyMap(k, d, np.eye(3), k,
                                      (intr.width, intr.height), cv2.CV_32FC1)
    return k, d, maps


class Inputs(Node):
    def __init__(self, args):
        super().__init__('orbbec336l_eye_to_hand_inputs')
        self.args, self.pipe, self.bus = args, None, None
        self.images = self.joints = 0
        self.last_image = self.last_joint = None
        self.started = time.monotonic()
        self.frame_id = 'orbbec336l_color_optical'
        self.fk = FK(args.urdf)
        self.tf = TransformBroadcaster(self)
        self.image_pub = self.create_publisher(Image, '/handeye/color/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/handeye/color/camera_info', 10)
        try:
            self.orbbec_context = Context()
            devices = self.orbbec_context.query_devices()
            if devices.get_count() == 0:
                raise RuntimeError('No Orbbec camera detected; connect Gemini 336L first')
            if not args.serial and devices.get_count() != 1:
                raise ValueError('Multiple Orbbec cameras detected; specify --serial')
            device = (devices.get_device_by_serial_number(args.serial) if args.serial
                      else devices.get_device_by_index(0))
            info = device.get_device_info()
            if '336' not in info.get_name():
                raise ValueError(f'Expected Gemini 336L; found {info.get_name()}')
            self.pipe = Pipeline(device)
            profiles = self.pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            profile = profiles.get_default_video_stream_profile()
            intr, distortion = profile.get_intrinsic(), profile.get_distortion()
            self.k, d, self.maps = camera_geometry(intr, distortion)
            self.width, self.height = intr.width, intr.height
            config = Config()
            config.enable_stream(profile)
            self.pipe.start(config)
            if not args.camera_only:
                self.bus = PassiveJoints(args.can)
                self.create_timer(.02, self.on_joint)
            write_json(args.session / 'inputs.json', dict(
                mode='eye-to-hand', camera_backend='Orbbec SDK v2',
                camera_name=info.get_name(), serial=info.get_serial_number(),
                resolution=[self.width, self.height], K=self.k.tolist(),
                raw_distortion=str(distortion.model), raw_coefficients=d.tolist(),
                published_distortion=[0.] * 5, rectified=True, camera_frame=self.frame_id,
                base_frame=self.fk.base, gripper_frame=self.fk.tip,
                can=None if args.camera_only else args.can,
                board=dict(columns=3, rows=4, dictionary='DICT_5X5_250',
                           first_id=0, marker_m=.060, separation_m=.006),
                no_motor_commands=True, camera_only=args.camera_only))
            self.create_timer(.03, self.on_frame)
            self.get_logger().info('Fixed-camera input ready. Camera images + passive CAN only; no motor commands.')
        except BaseException:
            self.close()
            raise

    def on_joint(self):
        now = time.monotonic()
        try:
            q, timing = self.bus.snapshot(now)
        except ValueError:
            return
        age = now - timing['joint_oldest_monotonic']
        t = self.fk(q)
        msg = TransformStamped()
        msg.header.stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=max(0., age))).to_msg()
        msg.header.frame_id, msg.child_frame_id = self.fk.base, self.fk.tip
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, t[:3, 3])
        rotation = cv2.Rodrigues(t[:3, :3])[0].reshape(3)
        angle = np.linalg.norm(rotation)
        xyz = rotation * (np.sin(angle / 2) / angle) if angle > 1e-12 else rotation / 2
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z = map(float, xyz)
        msg.transform.rotation.w = float(np.cos(angle / 2))
        self.tf.sendTransform(msg)
        self.joints += 1
        self.last_joint = now

    def on_frame(self):
        frames = self.pipe.wait_for_frames(1)
        color = frames.get_color_frame() if frames is not None else None
        if color is None:
            if time.monotonic() - (self.last_image or self.started) > 10:
                raise RuntimeError('No Orbbec color frame for 10s; check USB and restart this session')
            return
        raw = color_bgr(color)
        if raw.shape[:2] != (self.height, self.width):
            raise ValueError('Image resolution differs from its color intrinsics')
        image = cv2.remap(raw, *self.maps, interpolation=cv2.INTER_LINEAR)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = image.shape[:2]
        msg.encoding, msg.step = 'bgr8', msg.width * 3
        msg.data = array('B', image.tobytes())
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = msg.width, msg.height
        info.distortion_model, info.d = 'plumb_bob', [0.] * 5
        info.k, info.r = self.k.flatten().tolist(), np.eye(3).flatten().tolist()
        info.p = np.c_[self.k, np.zeros(3)].flatten().tolist()
        self.info_pub.publish(info)
        self.image_pub.publish(msg)
        self.images += 1
        self.last_image = time.monotonic()
        if self.images == 10:
            write_json(self.args.session / 'ready.json', dict(images=self.images))

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except RuntimeError:
                pass
            self.pipe = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can', default='can1', choices=['can1'])
    parser.add_argument('--serial')
    parser.add_argument('--camera-only', action='store_true')
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, default=ROOT / 'config/gravity/nero_standard_gripper_no_camera.urdf')
    parser.add_argument('--check-seconds', type=float, default=0.)
    args = parser.parse_args()
    args.session.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = None
    try:
        node = Inputs(args)
        end = time.monotonic() + args.check_seconds
        while rclpy.ok() and (args.check_seconds <= 0 or time.monotonic() < end):
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            write_json(args.session / 'summary.json', dict(images=node.images,
                joint_transforms=node.joints, no_motor_commands=True))
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
