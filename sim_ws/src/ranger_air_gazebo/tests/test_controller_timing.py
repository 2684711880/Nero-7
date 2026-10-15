"""Exercise the actual timer callback under skipped/jittered simulation ticks."""
import time
import unittest
from types import SimpleNamespace
import numpy as np
from ranger_air_gazebo.controller import AirController
from ranger_air_gazebo.kinematics import inverse


class Publisher:
    def publish(self, msg):
        self.last = msg


class TimerRegression(unittest.TestCase):
    def fake_node(self):
        n = SimpleNamespace(have_feedback=True, last_tick=0., stamp=0.,
                            last_cmd=time.monotonic(), last_feedback=time.monotonic(),
                            command_timeout=100., feedback_timeout=100.,
                            filtered=np.array([.15, 0., 0.]), target=np.zeros(3),
                            linear_acceleration=.15, angular_acceleration=.3,
                            angles=[0.]*4, steering_command=[0.]*4,
                            steering_rate=.8, alignment_tolerance=.12, alignment_stop_angle=.5,
                            steer_pub=Publisher(), wheel_pub=Publisher())
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(n.stamp*1e9)))
        return n

    def test_stop_time_does_not_depend_on_callback_count(self):
        # Original fixed-20-ms code leaves 0.09 m/s after 1 second at 20 Hz.
        for step in [.02, .05]:
            n = self.fake_node()
            for k in range(round(1./step)):
                n.stamp = (k+1)*step
                AirController.tick(n)
            self.assertLess(abs(n.filtered[0]), 1e-8)
            self.assertLess(max(abs(x) for x in n.wheel_pub.last.data), 1e-6)

    def drive_at_error(self, error):
        n = self.fake_node()
        n.filtered = n.target = np.array([.06, 0., .12])
        desired, _ = inverse(*n.target, [0.] * 4)
        n.angles = [a-error for a in desired]
        n.steering_command = n.angles.copy()
        n.stamp = .02
        AirController.tick(n)
        return n

    def test_combined_drive_does_not_switch_off_at_small_alignment_boundary(self):
        # Reproduce the actual timer seam: a 0.002-rad tracking fluctuation
        # around the old 0.12-rad gate must not switch all four drive wheels off.
        before = self.drive_at_error(.119)
        after = self.drive_at_error(.121)
        initial = np.linalg.norm(before.wheel_pub.last.data)
        following = np.linalg.norm(after.wheel_pub.last.data)
        self.assertGreater(initial, .1)
        self.assertGreater(following/initial, .99)

    def test_large_alignment_error_still_prevents_drive(self):
        n = self.drive_at_error(.6)
        self.assertTrue(all(v == 0. for v in n.wheel_pub.last.data))

    def test_alignment_taper_is_monotonic_and_keeps_wheel_ratios(self):
        baseline = np.array(self.drive_at_error(0.).wheel_pub.last.data)
        scales = []
        for error in (.12, .2, .3, .4, .5):
            rates = np.array(self.drive_at_error(error).wheel_pub.last.data)
            ratios = rates/baseline
            np.testing.assert_allclose(ratios, ratios[0], atol=1e-9)
            scales.append(ratios[0])
        self.assertAlmostEqual(scales[0], 1.)
        self.assertAlmostEqual(scales[-1], 0.)
        self.assertTrue(all(a > b for a,b in zip(scales, scales[1:])))

    def test_stale_feedback_or_command_stops_all_wheels(self):
        for field in ('last_feedback', 'last_cmd'):
            n = self.drive_at_error(0.)
            setattr(n, field, -float('inf'))
            AirController.tick(n)
            self.assertTrue(all(v == 0. for v in n.wheel_pub.last.data))
            np.testing.assert_array_equal(n.filtered, np.zeros(3))

    def test_near_stationary_wheel_does_not_block_other_three(self):
        # Replay b3 at t=12.9 s from validation/base_smoothness/compensation_only.json.
        # RL needs only 7 mm/s near the instantaneous rotation center; its
        # 0.759-rad steering change used to stop the other three wheels too.
        n = self.fake_node()
        n.filtered = n.target = np.array([.00787224413, .01575063873, .08831493221])
        n.angles = [4.84573865217, -2.15617295298, 5.71812728477, 3.01413170188]
        n.steering_command = n.angles.copy()
        n.stamp = .02
        _, expected = inverse(*n.target, n.angles)
        AirController.tick(n)
        self.assertGreater(np.linalg.norm(n.wheel_pub.last.data)/np.linalg.norm(expected), .9)


if __name__ == '__main__':
    unittest.main()
