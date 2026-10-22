"""Small, feedback-based NERO shoulder swing for the Gazebo keyboard only."""
import math
import time

from builtin_interfaces.msg import Duration
from rclpy.qos import qos_profile_sensor_data
from moveit_msgs.srv import GetStateValidity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmSwing:
    AMPLITUDE = 0.30
    PERIOD = 8.0

    def __init__(self, node):
        self.node = node
        self.enabled = False
        self.feedback = None
        self.feedback_at = 0.0
        self.clock_at = 0.0
        self.clock_ns = 0
        self.future = None
        self.checks = []
        self.finish_ns = 0
        self.origin = None
        self.pubs = {s: node.create_publisher(JointTrajectory,
                     f'/{s}_arm_controller/joint_trajectory', 10)
                     for s in ('left', 'right')}
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')
        self.subs = [node.create_subscription(JointState, '/joint_states', self.joints, 10),
                     node.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)]

    def joints(self, msg):
        self.feedback = msg
        self.feedback_at = time.monotonic()

    def clock(self, msg):
        ns = msg.clock.sec * 10**9 + msg.clock.nanosec
        if ns > self.clock_ns:
            self.clock_at = time.monotonic()
        elif ns < self.clock_ns:
            self.stop()
        self.clock_ns = ns

    def fresh(self):
        now = time.monotonic()
        return (self.feedback is not None and now - self.feedback_at < 1.0
                and now - self.clock_at < 1.0)

    def toggle(self):
        if self.enabled:
            self.stop()
            return
        if not self.fresh() or not self.validity.service_is_ready():
            self.node.get_logger().warning('Swing needs live Gazebo /clock, joint feedback and MoveIt.')
            return
        if any(p.get_subscription_count() == 0 for p in self.pubs.values()):
            self.node.get_logger().warning('Both arm trajectory controllers must be running.')
            return
        q = dict(zip(self.feedback.name, self.feedback.position))
        for side in self.pubs:
            for j in range(1, 8):
                v = q.get(f'{side}_joint{j}', float('nan'))
                limit = .40 if j == 1 else .25
                outside_relaxed = not 1.0 <= v <= 1.35 if j == 2 else abs(v) > limit
                if not math.isfinite(v) or outside_relaxed:
                    allowed = '[1.00, 1.35]' if j == 2 else f'[-{limit:.2f}, {limit:.2f}]'
                    self.node.get_logger().warning(
                        f'Swing start rejected: {side}_joint{j}={v:.3f} rad; allowed {allowed}.')
                    return
        self.start = q.copy()
        self.origin = q.copy()
        for side in self.pubs:
            self.origin[f'{side}_joint1'] = 0.0
        self.enabled = True
        self.finish_ns = 0
        self.node.get_logger().info('Arm swing ON: alternating fore/aft, +/-0.30 rad joint1, 8 s/cycle; checking path.')

    def stop(self):
        was_enabled = self.enabled
        self.enabled = False
        self.checks = []
        if self.future is not None:
            self.future.cancel()
            self.future = None
        if was_enabled and self.fresh():
            q = dict(zip(self.feedback.name, self.feedback.position))
            for side, pub in self.pubs.items():
                msg = JointTrajectory()
                msg.joint_names = [f'{side}_joint{j}' for j in range(1, 8)]
                if all(k in q for k in msg.joint_names):
                    msg.points = [JointTrajectoryPoint(
                        positions=[q[k] for k in msg.joint_names], velocities=[0.0]*7,
                        time_from_start=Duration(sec=0, nanosec=500000000))]
                    pub.publish(msg)
        if was_enabled:
            self.node.get_logger().info('Arm swing OFF; holding current pose.')

    def tick(self):
        if not self.enabled:
            return
        if not self.fresh():
            self.stop()
            self.node.get_logger().warning('Swing stopped: simulation clock or feedback stale.')
            return
        if self.finish_ns and self.clock_ns < self.finish_ns:
            return
        if self.future is not None:
            if not self.future.done():
                if time.monotonic() - self.request_at > 3.0:
                    self.stop()
                    self.node.get_logger().warning('Swing stopped: collision check timed out.')
                return
            result = self.future.result()
            self.future = None
            if result is None or not result.valid:
                self.stop()
                self.node.get_logger().warning('Swing stopped: MoveIt rejected the sampled path.')
                return
            if not self.checks:
                self.publish_cycle()
                return
        if not self.checks:
            actual = dict(zip(self.feedback.name, self.feedback.position))
            if any(abs(actual.get(k, float('inf')) - (self.start[k] if not self.finish_ns else self.origin[k])) > .08
                   for side in self.pubs for k in [f'{side}_joint{j}' for j in range(1, 8)]):
                self.stop()
                self.node.get_logger().warning('Swing stopped: arm did not return to cycle origin.')
                return
            self.start = actual.copy()
            # Check the same sampled states as the commanded resume/cycle trajectory.
            for i in range(33):
                positions, _ = self.cycle_state(i*self.PERIOD/32)
                request = GetStateValidity.Request()
                request.robot_state.is_diff = True
                request.robot_state.joint_state.name = list(actual)
                request.robot_state.joint_state.position = [positions.get(k, v) for k, v in actual.items()]
                self.checks.append(request)
        self.future = self.validity.call_async(self.checks.pop(0))
        self.request_at = time.monotonic()

    def cycle_state(self, t):
        phase = 2*math.pi*t/self.PERIOD
        wave = self.AMPLITUDE*math.sin(phase)**3
        wave_v = self.AMPLITUDE*3*math.sin(phase)**2*math.cos(phase)*2*math.pi/self.PERIOD
        # Blend from the measured stopped angle over the first quarter-cycle.
        # Convex blending keeps the resumed motion within the commanded envelope.
        duration = self.PERIOD/4
        u = min(1.0, max(0.0, t/duration))
        h = 10*u**3-15*u**4+6*u**5
        dh = (30*u**2-60*u**3+30*u**4)/duration if t < duration else 0.0
        q, v = self.origin.copy(), {k: 0.0 for k in self.origin}
        for side in self.pubs:
            k = f'{side}_joint1'
            q[k] = (1-h)*self.start[k]+h*wave
            v[k] = dh*(wave-self.start[k])+h*wave_v
        return q, v

    def publish_cycle(self):
        for side, pub in self.pubs.items():
            msg = JointTrajectory()
            msg.joint_names = [f'{side}_joint{j}' for j in range(1, 8)]
            for i in range(33):
                t = i*self.PERIOD/32
                q, v = self.cycle_state(t)
                ns = round(t*1e9)
                msg.points.append(JointTrajectoryPoint(
                    positions=[q[k] for k in msg.joint_names],
                    velocities=[v[k] for k in msg.joint_names],
                    time_from_start=Duration(sec=ns//10**9, nanosec=ns%10**9)))
            pub.publish(msg)
        self.finish_ns = self.clock_ns + round((self.PERIOD+.25)*1e9)
