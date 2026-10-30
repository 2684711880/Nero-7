"""Model geometry and official SDK preparation flow; no hardware connection."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import tempfile
import json
import threading
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import joycon_ready as ready

TAUGHT = {'left': [3, 82, -87, 85, 4, 2, -5], 'right': [-2, 84, 88, 83, -3, 1, 6]}
INITIAL = [0, 85, 0, 5, 0, 0, 0]


class Clock:
    value = 1000.

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class Arm:
    def __init__(self, clock, initial):
        self.clock = clock
        self.q = np.deg2rad(initial).tolist()
        self.targets = []
        self.closed = False
        self.speed = None
        self.fail_after_motion = False

    def connect(self): pass
    def enable(self): return True
    def disconnect(self): self.closed = True
    def set_speed_percent(self, percent): self.speed = percent

    def get_joint_angles(self):
        return SimpleNamespace(msg=self.q, timestamp=self.clock.now() - (
            1 if self.fail_after_motion and self.targets else 0))

    def move_j(self, q):
        self.targets.append(q)
        self.q = q


class JointReader:
    error = None

    def __init__(self, arm, clock):
        self.arm, self.clock = arm, clock
        self.lock = threading.Lock()

    @property
    def history(self):
        timestamp = self.clock.now() - (1 if self.arm.fail_after_motion and self.arm.targets else 0)
        return [(timestamp, timestamp, np.asarray(self.arm.q))]

    def close(self): pass


class ReadyTests(unittest.TestCase):
    def run_arms(self, arms, clock, returning=False):
        with patch.object(ready, 'load_ready_degrees', return_value=TAUGHT), \
             patch.object(ready, 'open_joint_feedback', side_effect=lambda can: JointReader(arms[can], clock)), \
             patch('pyAgxArm.create_agx_arm_config', side_effect=lambda **kw: kw), \
             patch('pyAgxArm.AgxArmFactory.create_arm', side_effect=lambda c: arms[c['channel']]), \
             patch('subprocess.check_output', return_value=''), \
             patch.object(ready.time, 'time', clock.now), \
             patch.object(ready.time, 'monotonic', clock.now), \
             patch.object(ready.time, 'sleep', clock.sleep):
            ready.execute(returning=returning)

    def test_recorded_targets_are_used_without_geometric_override(self):
        report = ready.model_report(TAUGHT)
        for side in TAUGHT:
            np.testing.assert_allclose(report[side]['joints_deg'], TAUGHT[side])

    def test_prepare_at_ten_percent_uses_recorded_target(self):
        clock = Clock()
        arms = {can: Arm(clock, INITIAL) for can in ready.CHANNELS.values()}
        self.run_arms(arms, clock)
        for side, can in ready.CHANNELS.items():
            arm = arms[can]
            self.assertEqual(arm.speed, 10)
            self.assertEqual(len(arm.targets), 1)
            np.testing.assert_allclose(arm.targets[0], np.deg2rad(TAUGHT[side]))
            self.assertTrue(arm.closed)

    def test_missing_calibration_refuses_before_connecting(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(ready, 'POSE_FILE', Path(directory) / 'absent.json'), \
             patch('pyAgxArm.AgxArmFactory.create_arm') as create:
            with self.assertRaisesRegex(RuntimeError, '尚未记录'):
                ready.execute()
            create.assert_not_called()

    def test_load_recorded_radians_and_verify_can_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'ready.json'
            data = dict(schema=1, source='passive_can_joint_feedback', arms={
                side: dict(can=can, joints_rad=np.deg2rad(TAUGHT[side]).tolist())
                for side, can in ready.CHANNELS.items()})
            path.write_text(json.dumps(data))
            with patch.object(ready, 'POSE_FILE', path):
                result = ready.load_ready_degrees()
                np.testing.assert_allclose(result['left'], TAUGHT['left'])
                data['arms']['left']['can'] = 'can0'
                path.write_text(json.dumps(data))
                with self.assertRaisesRegex(RuntimeError, '映射'):
                    ready.load_ready_degrees()

    def test_already_ready_has_no_repeated_motion(self):
        clock = Clock()
        arms = {can: Arm(clock, TAUGHT[side]) for side, can in ready.CHANNELS.items()}
        self.run_arms(arms, clock)
        self.assertTrue(all(not arm.targets and arm.closed for arm in arms.values()))

    def test_stale_feedback_does_not_advance_to_next_stage(self):
        clock = Clock()
        arms = {can: Arm(clock, INITIAL) for can in ready.CHANNELS.values()}
        arms['can1'].fail_after_motion = True
        with self.assertRaisesRegex(RuntimeError, '过期'):
            self.run_arms(arms, clock)
        self.assertEqual(len(arms['can1'].targets), 1)
        np.testing.assert_allclose(arms['can1'].targets[0], np.deg2rad(TAUGHT['left']))
        self.assertTrue(all(arm.closed for arm in arms.values()))

    def test_return_from_teleop_pose_has_one_target_at_ten_percent(self):
        clock = Clock()
        initial = [10, 60, 20, 60, 10, 10, 10]
        arms = {can: Arm(clock, initial) for can in ready.CHANNELS.values()}
        self.run_arms(arms, clock, returning=True)
        for side, can in ready.CHANNELS.items():
            self.assertEqual(arms[can].speed, 10)
            self.assertEqual(len(arms[can].targets), 1)
            np.testing.assert_allclose(arms[can].targets[0], np.deg2rad(TAUGHT[side]))

    def test_return_refuses_invalid_current_joint_before_any_motion(self):
        clock = Clock()
        arms = {can: Arm(clock, [0, 180, 0, 0, 0, 0, 0]) for can in ready.CHANNELS.values()}
        with self.assertRaisesRegex(RuntimeError, '限位'):
            self.run_arms(arms, clock, returning=True)
        self.assertTrue(all(not arm.targets and arm.closed for arm in arms.values()))

    def test_j6_near_official_lower_boundary_is_valid(self):
        clock = Clock()
        initial = INITIAL.copy(); initial[5] = -41.958
        arms = {can: Arm(clock, initial) for can in ready.CHANNELS.values()}
        self.run_arms(arms, clock, returning=True)
        self.assertTrue(all(len(arm.targets) == 1 for arm in arms.values()))

    def test_j6_outside_official_limit_is_rejected(self):
        clock = Clock()
        initial = INITIAL.copy(); initial[5] = -42.1
        arms = {can: Arm(clock, initial) for can in ready.CHANNELS.values()}
        with self.assertRaisesRegex(RuntimeError, '限位'):
            self.run_arms(arms, clock, returning=True)
        self.assertTrue(all(not arm.targets for arm in arms.values()))

    def test_sdk_partial_zero_array_is_not_accepted_without_complete_frames(self):
        robot = SimpleNamespace(get_joint_angles=lambda: SimpleNamespace(msg=[0.]*7, timestamp=ready.time.time()),
                                _joycon_joint_feedback=SimpleNamespace(error=None, lock=threading.Lock(), history=[]))
        with self.assertRaisesRegex(RuntimeError, '七轴'):
            ready.angles(robot)

    def test_existing_driver_prevents_return(self):
        with patch.object(ready, 'load_ready_degrees', return_value=TAUGHT), \
             patch('subprocess.check_output', return_value='/ros/lib/agx_arm_ctrl/agx_arm_ctrl_single'), \
             patch('pyAgxArm.AgxArmFactory.create_arm') as create:
            with self.assertRaisesRegex(RuntimeError, '控制进程'):
                ready.execute(returning=True)
            create.assert_not_called()

    def test_record_each_side_separately_and_require_both(self):
        now = 1000.
        class Reader:
            def __init__(self, channel):
                self.lock = threading.Lock()
                side = 'left' if channel == 'can1' else 'right'
                self.history = [(t, t + .001, np.deg2rad(TAUGHT[side]))
                                for t in np.linspace(now - 1.9, now - .005, 60)]
            def snapshot(self, timestamp): pass
            def close(self): pass
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(ready, 'POSE_FILE', Path(directory) / 'ready_pose.json'), \
             patch('handeye_charuco.PassiveJoints', Reader), \
             patch.object(ready.time, 'sleep'), \
             patch.object(ready.time, 'monotonic', return_value=now):
            ready.record_pose(('left',))
            left = json.loads(ready.POSE_FILE.read_text())['arms']['left']
            with self.assertRaisesRegex(RuntimeError, 'right.*尚未记录'):
                ready.load_ready_degrees()
            ready.record_pose(('right',))
            saved = json.loads(ready.POSE_FILE.read_text())
            self.assertEqual(saved['arms']['left'], left)
            np.testing.assert_allclose(ready.load_ready_degrees()['right'], TAUGHT['right'])
            self.assertEqual(len(list(Path(directory).glob('*.bak.json'))), 1)

    def test_mirror_uses_verified_wrist_signs_and_marks_derived_target(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(ready, 'POSE_FILE', Path(directory) / 'ready_pose.json'):
            data = dict(schema=1, source='passive_can_joint_feedback', arms={
                'left': dict(can='can1', captured_at='test', joints_rad=np.deg2rad(TAUGHT['left']).tolist())})
            ready.POSE_FILE.write_text(json.dumps(data))
            ready.mirror_left_to_right()
            saved = json.loads(ready.POSE_FILE.read_text())
            target = ready.load_ready_degrees()['right']
            np.testing.assert_allclose(target, np.asarray(TAUGHT['left']) * [-1, 1, -1, 1, -1, -1, 1])
            self.assertEqual(saved['arms']['right']['source'], 'urdf_mirror_of_left')
            self.assertFalse(saved['arms']['right']['hardware_verified'])
            self.assertLess(saved['arms']['right']['model_orientation_error_deg'], .1)


if __name__ == '__main__':
    unittest.main()
