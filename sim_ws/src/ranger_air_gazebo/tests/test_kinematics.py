"""Coordinate/sign regression checks independent of Gazebo."""
import math
import unittest
import numpy as np
from ranger_air_gazebo.kinematics import inverse, forward, wrap, DRIVE_SIGN, STEER_SIGN


class KinematicsTest(unittest.TestCase):
    def test_forward_joint_signs(self):
        q, w = inverse(.2, 0., 0., [0.] * 4)
        np.testing.assert_allclose(q, 0., atol=1e-10)
        np.testing.assert_allclose(np.sign(w), DRIVE_SIGN)

    def test_roundtrip(self):
        for twist in [(0.2, 0, 0), (-.2, 0, 0), (0, .2, 0), (0, -.2, 0),
                      (0, 0, .4), (0, 0, -.4), (.2, -.1, .3)]:
            q, w = inverse(*twist, [0.] * 4)
            np.testing.assert_allclose(forward(q, w), twist, atol=1e-10)

    def test_shortest_turn_and_zero_hold(self):
        q0 = [2*math.pi + .3, -4*math.pi, .1, -.1]
        q, w = inverse(-.2, 0., 0., q0)
        self.assertTrue(all(abs(a-b) <= math.pi/2 for a, b in zip(q, q0)))
        q, w = inverse(0., 0., 0., q0)
        self.assertEqual(q, q0)
        self.assertEqual(w, [0.] * 4)


if __name__ == '__main__':
    unittest.main()
