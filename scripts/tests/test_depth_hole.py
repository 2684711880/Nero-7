"""Geometry-level detector regression using synthetic depth, with no camera or ROS."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from depth_hole import detect_depth_hole


class DepthHoleTest(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
        self.yy, self.xx = np.mgrid[:480, :640]

    def scene(self, centers):
        depth = np.full((480, 640), .5)
        for x, y in centers:
            depth[(self.xx-x)**2 + (self.yy-y)**2 < 45**2] = 1.
        return depth

    def test_recovers_90mm_hole_from_depth(self):
        result = detect_depth_hole(self.scene([(320, 240)]), self.k)
        np.testing.assert_allclose(result['center'], [0., 0., .5], atol=.001)
        np.testing.assert_allclose(result['normal'], [0., 0., -1.], atol=.001)
        self.assertAlmostEqual(2*result['radius'], .09, delta=.003)

    def test_solid_board_is_not_a_hole(self):
        with self.assertRaisesRegex(ValueError, 'no enclosed aperture'):
            detect_depth_hole(self.scene([]), self.k)

    def test_multiple_holes_are_ambiguous(self):
        with self.assertRaisesRegex(ValueError, 'multiple apertures'):
            detect_depth_hole(self.scene([(220, 240), (420, 240)]), self.k)


if __name__ == '__main__':
    unittest.main()
