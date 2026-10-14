"""Exercise the actual timer callback under skipped/jittered simulation ticks."""
import time
import unittest
from types import SimpleNamespace
import numpy as np
from ranger_air_gazebo.controller import AirController


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
                            steering_rate=.8, alignment_tolerance=.12,
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


if __name__ == '__main__':
    unittest.main()
