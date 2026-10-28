"""Offline geometry/protocol regressions; never opens a camera or CAN interface."""
import importlib.util
from pathlib import Path
import struct
import threading
import tempfile
import sys
from collections import deque
import unittest

import cv2
import numpy as np

SPEC = importlib.util.spec_from_file_location('handeye', Path(__file__).parents[1] / 'handeye_charuco.py')
h = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(h)
sys.path.insert(0, str(Path(__file__).parents[1]))


class HandeyeTests(unittest.TestCase):
    def test_rgb_calibration_preserves_resolution_at_lower_frame_rate(self):
        # Actual USB2 D405 reports 720p RGB at 15fps, while 30fps ends at VGA.
        profiles = [(640,480,30), (480,270,30), (848,480,10), (1280,720,5), (1280,720,15)]
        self.assertEqual(h.select_color_profile(profiles), (1280,720,15))
        self.assertEqual(h.select_color_profile(profiles+[(1280,720,30)]), (1280,720,30))
        with self.assertRaises(ValueError):
            h.select_color_profile([])

    def test_inverse_brown_camera_uses_rectified_zero_distortion(self):
        import pyrealsense2 as rs
        intr = rs.intrinsics()
        intr.width, intr.height = 128, 96
        intr.fx, intr.fy, intr.ppx, intr.ppy = 100., 101., 63., 47.
        intr.model = rs.distortion.inverse_brown_conrady
        intr.coeffs = [-.05, .05, .0001, -.0013, -.016]
        k, dist, maps = h.camera_geometry(intr)
        np.testing.assert_allclose(dist, np.zeros(5))
        np.testing.assert_allclose(k, [[100, 0, 63], [0, 101, 47], [0, 0, 1]])
        self.assertEqual(maps[0].shape, (96, 128))
        # Every remapped source pixel must deproject back to the requested ideal ray.
        for x, y in [(20, 20), (60, 40), (100, 70)]:
            ray = rs.rs2_deproject_pixel_to_point(intr, [float(maps[0][y,x]), float(maps[1][y,x])], 1.)
            ideal = np.array(ray[:2]) / ray[2] * [intr.fx, intr.fy] + [intr.ppx, intr.ppy]
            np.testing.assert_allclose(ideal, [x, y], atol=.04)

    def make_samples(self):
        rng = np.random.default_rng(42)
        x = h.transform(cv2.Rodrigues(np.array([.2, -.1, .3]))[0], [.04, -.03, .08])
        fixed_board = h.transform(cv2.Rodrigues(np.array([.1, .2, -.3]))[0], [.5, .1, .3])
        samples = []
        for _ in range(20):
            g = h.transform(cv2.Rodrigues(rng.normal(0, .6, 3))[0], rng.uniform(-.3, .3, 3))
            c = np.linalg.inv(x) @ np.linalg.inv(g) @ fixed_board
            samples.append(dict(T_base_gripper=g.tolist(), T_camera_board=c.tolist()))
        return x, samples

    def test_recovers_camera_to_gripper_direction_and_metres(self):
        x, samples = self.make_samples()
        result = h.solve(samples)
        np.testing.assert_allclose(result['T_gripper_camera'], x, atol=1e-10)
        np.testing.assert_allclose(np.array(result['T_camera_gripper']) @ x, np.eye(4), atol=1e-10)
        self.assertLess(result['training_board_position_rms_mm'], 1e-8)

    def test_rejects_single_axis_rotations(self):
        _, samples = self.make_samples()
        for i, s in enumerate(samples):
            s['T_base_gripper'] = h.transform(cv2.Rodrigues(np.array([0., 0., i * .04]))[0]).tolist()
        with self.assertRaisesRegex(ValueError, '退化'):
            h.solve(samples)

    def test_joint_signed_big_endian_seventh_axis(self):
        values = [-90000, 45000, -1000, 2000, 3000, -4000, 7000]
        frames = {}
        for i, cid in enumerate(h.JOINT_IDS):
            pair = values[i*2:i*2+2]
            if len(pair) == 1:
                pair.append(999999)
            frames[cid] = (0., struct.pack('>ii', *pair))
        np.testing.assert_allclose(h.decode_joints(frames), np.deg2rad(np.array(values)/1000.))

    def test_capture_rejects_stale_or_moving_feedback(self):
        bus = object.__new__(h.PassiveJoints)
        bus.error = None
        bus.lock = threading.Lock()
        bus.history = deque((t-.005, t, np.zeros(7)) for t in np.linspace(9.4, 10., 100))
        q, _ = bus.snapshot(10.)
        np.testing.assert_allclose(q, np.zeros(7))
        with self.assertRaisesRegex(ValueError, '100ms'):
            bus.snapshot(10.2)
        bus.history[-1] = (9.995, 10., np.ones(7) * .02)
        with self.assertRaisesRegex(ValueError, '变化'):
            bus.snapshot(10.)

    def test_held_out_validation_reports_error_without_refitting(self):
        x, samples = self.make_samples()
        result = h.solve(samples)
        meta = {key: 'same' for key in ('camera_serial', 'board', 'K', 'distortion', 'can', 'urdf_sha256', 'base_frame', 'gripper_frame')}
        with tempfile.TemporaryDirectory() as tmp:
            ref, new = Path(tmp) / 'ref', Path(tmp) / 'new'
            ref.mkdir(); new.mkdir()
            h.write_json(ref / 'session.json', meta)
            h.write_json(new / 'session.json', meta)
            h.write_json(ref / 'result.json', result)
            sample = dict(samples[0], image='new.png')
            # An independently measured board position 10 mm off the reference.
            g = np.array(sample['T_base_gripper'])
            g[0, 3] += .01
            sample['T_base_gripper'] = g.tolist()
            h.write_json(new / 'samples.json', [sample])
            report = h.validate(ref, new)
            self.assertAlmostEqual(report['position_rms_mm'], 10., places=7)
            meta['can'] = 'different'
            h.write_json(new / 'session.json', meta)
            with self.assertRaisesRegex(ValueError, 'can'):
                h.validate(ref, new)

    def test_detects_all_corners_and_pose_of_rendered_board(self):
        config = dict(squares_x=7, squares_y=5, square_m=.04, marker_m=.03, dictionary='DICT_5X5_50')
        b = h.Board(config)
        image = cv2.copyMakeBorder(b.board.draw((980, 700)), 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        k = np.array([[1000., 0, image.shape[1]/2], [0, 1000., image.shape[0]/2], [0, 0, 1.]])
        result, _ = b.detect(image, k, np.zeros(5))
        self.assertEqual(len(result['corner_ids']), 24)
        self.assertLess(result['reprojection_rms_px'], .1)

    def test_fk_zero_and_joint1_rotation(self):
        fk = h.FK(h.ROOT / 'config/gravity/nero_standard_gripper_no_camera.urdf')
        a = fk(np.zeros(7))
        q = np.zeros(7)
        q[0] = .3
        b = fk(q)
        # joint1 axis is the base Z; base-to-joint1 translation is along Z.
        rot = h.transform(cv2.Rodrigues(np.array([0., 0., .3]))[0])
        np.testing.assert_allclose(b, rot @ a, atol=1e-7)


if __name__ == '__main__':
    unittest.main()
