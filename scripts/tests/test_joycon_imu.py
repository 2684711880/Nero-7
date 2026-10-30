import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from joycon_imu import JoyconIMU
from joycon_move_p_dual import orientation_step, MAX_ROTATION_STEP
from geometry_msgs.msg import PoseStamped
from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class TestIMU(unittest.TestCase):
    def sensor(self):
        obj = JoyconIMU.__new__(JoyconIMU)
        obj.orientation = Rotation.identity()
        obj.path = 'mock'
        obj.bias = np.zeros(3)
        obj.last_timestamp = None
        obj.last_event_time = None
        obj.sample_dt = .005
        return obj

    def test_bias_and_motion(self):
        imu = self.sensor()
        bias = np.array([.01, -.02, .03])
        imu.zero(np.tile(bias, (200, 1)))
        imu.update(100000, bias)
        for i in range(1, 101):
            imu.update(100000+i*5000, bias+np.array([0., 0., math.radians(20)]))
        np.testing.assert_allclose(imu.orientation.as_rotvec(), [0,0,math.radians(10)], atol=1e-8)

    def test_rejects_moving_calibration(self):
        imu = self.sensor()
        values = np.tile([[-.1,0,0],[.1,0,0]], (100,1))
        with self.assertRaises(RuntimeError): imu.zero(values)

    def test_timestamp_wrap(self):
        imu = self.sensor()
        imu.update(0xfffff000, np.zeros(3))
        imu.update((0xfffff000+5000)&0xffffffff, [0, 0, math.radians(20)])
        self.assertAlmostEqual(imu.orientation.as_rotvec()[2], math.radians(.1))

    def test_lost_samples_skip_integration(self):
        imu = self.sensor()
        imu.update(100000, np.zeros(3))
        imu.update(800000, [0, 0, math.radians(20)])
        np.testing.assert_allclose(imu.orientation.as_rotvec(), [0, 0, 0])
        with self.assertRaisesRegex(RuntimeError, '超过 2.0s'):
            imu.update(2900000, np.zeros(3))

    def test_orientation_world_mapping_and_bounded_step(self):
        pose = PoseStamped(); pose.pose.orientation.w = 1.0
        for roll in (-1.57, 1.57):
            mount = Rotation.from_euler('xyz', [roll, -1.57, 0])
            self.assertIsNone(orientation_step(pose, pose, Rotation.identity(), mount))
            command = orientation_step(pose, pose, Rotation.from_euler('x', 10, degrees=True), mount)
            base_step = Rotation.from_quat(command).as_rotvec()
            world_step = mount.apply(base_step)
            np.testing.assert_allclose(world_step, [MAX_ROTATION_STEP, 0, 0], atol=1e-10)
            target = PoseStamped()
            target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = map(float, command)
            sdk = Mock(); sdk.get_tcp2flange_pose.side_effect = lambda x: x
            node = SimpleNamespace(agx_arm=sdk, _check_can_control=lambda: True)
            node._create_pose_cmd = lambda value: AgxArmRosNode._create_pose_cmd(node, value)
            AgxArmRosNode._move_p_callback(node, target)
            sdk.move_p.assert_called_once()
            actual = Rotation.from_euler('xyz', sdk.move_p.call_args.args[0][3:]).as_quat()
            np.testing.assert_allclose(abs(np.dot(actual, command)), 1., atol=1e-10)

    def test_double_speed_kernel_clock_does_not_double_gyro_integral(self):
        imu = self.sensor()
        for packet in range(67):
            for sample in range(3):
                host = 10.0 + packet * .015 + sample * .00001
                fake_kernel_us = int((2*packet*.015 + sample*.005) * 1e6)
                imu.update(fake_kernel_us, [0., 0., math.radians(20)], host)
        self.assertAlmostEqual(math.degrees(imu.orientation.as_rotvec()[2]), 20., delta=.3)

    def test_real_event_gap_does_not_integrate_missing_motion(self):
        imu = self.sensor()
        imu.update(0, [0.,0.,0.], 10.0)
        imu.update(50000, [0.,0.,math.radians(50)], 10.5)
        np.testing.assert_allclose(imu.orientation.as_rotvec(), [0,0,0])
        with self.assertRaisesRegex(RuntimeError, '超过 2.0s'):
            imu.update(100000, [0.,0.,0.], 12.6)

    def test_large_rotation_tracks_until_ninety_degree_hard_limit(self):
        for degrees in (45, 80, 120):
            anchor = PoseStamped(); anchor.pose.orientation.w = 1.
            pose = PoseStamped(); pose.pose.orientation.w = 1.
            hand = Rotation.from_euler('x', degrees, degrees=True)
            mount = Rotation.from_euler('xyz', [-1.57, -1.57, 0])
            for _ in range(150):
                q = orientation_step(pose, anchor, hand, mount)
                if q is None:
                    break
                old = pose.pose.orientation
                previous = Rotation.from_quat([old.x, old.y, old.z, old.w])
                self.assertLessEqual((previous.inv() * Rotation.from_quat(q)).magnitude(),
                                     MAX_ROTATION_STEP + 1e-10)
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            q = pose.pose.orientation
            actual = Rotation.from_quat([q.x, q.y, q.z, q.w])
            self.assertAlmostEqual(math.degrees(actual.magnitude()), min(degrees, 90), delta=.51)


if __name__ == '__main__': unittest.main()
