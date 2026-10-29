import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from real_hole_geometry import transform_depth_observation, flange_goal


class GeometryTest(unittest.TestCase):
    def test_depth_to_color_then_handeye_and_base(self):
        base_flange = np.eye(4)
        base_flange[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        base_flange[:3, 3] = [1, 2, 3]
        flange_color = np.eye(4)
        flange_color[:3, 3] = [.1, 0, 0]
        color_depth = np.eye(4)
        color_depth[:3, 3] = [0, .02, 0]
        center, normal = transform_depth_observation(base_flange, flange_color, color_depth, [0, 0, .4], [0, 0, -2])
        np.testing.assert_allclose(center, [.98, 2.1, 3.4])
        np.testing.assert_allclose(normal, [0, 0, -1])

    def test_invalid_transform_is_rejected(self):
        invalid = np.eye(4)
        invalid[0, 0] = 2
        with self.assertRaises(ValueError):
            transform_depth_observation(invalid, np.eye(4), np.eye(4), [0, 0, 1], [0, 0, -1])

    def test_tool_tip_and_axis_at_approach(self):
        goal = flange_goal([.4, .1, .2], [0, 0, -1], np.eye(3), [.01, 0, .18], [0, 0, 1], .1)
        np.testing.assert_allclose((goal @ [.01, 0, .18, 1])[:3], [.4, .1, .1], atol=1e-9)
        np.testing.assert_allclose(goal[:3, :3] @ [0, 0, 1], [0, 0, 1], atol=1e-9)

    def test_opposite_axis_rotation(self):
        goal = flange_goal([0, 0, 0], [0, 0, 1], np.eye(3), [0, 0, .1], [0, 0, 1], .1)
        np.testing.assert_allclose(goal[:3, :3] @ [0, 0, 1], [0, 0, -1], atol=1e-8)


if __name__ == '__main__':
    unittest.main()
