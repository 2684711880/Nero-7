"""Feedback-based, collision-checked height commands for the simulated upper body."""
import math
import time
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetStateValidity


class LiftControl:
    JOINT = 'upper_body_lift_joint'
    UPPER = .80

    def __init__(self, node):
        self.node = node
        self.feedback = None
        self.feedback_at = 0.
        self.clock_at = 0.
        self.clock_ns = 0
        self.base_stopped = False
        self.base_at = 0.
        self.active = False
        self.future = None
        self.checks = []
        self.finish = 0
        self.pub = node.create_publisher(JointTrajectory, '/upper_body_lift_controller/joint_trajectory', 10)
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')
        self.subs = [node.create_subscription(JointState,'/joint_states',self.joints,10),
                     node.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data),
                     node.create_subscription(Odometry,'/odom',self.odometry,10)]

    def odometry(self, msg):
        v=msg.twist.twist
        self.base_stopped = max(abs(v.linear.x),abs(v.linear.y),abs(v.angular.z)) < .02
        self.base_at = time.monotonic()

    def joints(self, msg):
        if self.JOINT in msg.name:
            self.feedback = msg; self.feedback_at = time.monotonic()

    def clock(self, msg):
        t = msg.clock.sec*10**9+msg.clock.nanosec
        if t > self.clock_ns: self.clock_at = time.monotonic()
        if t < self.clock_ns: self.stop()
        self.clock_ns = t

    def fresh(self):
        return (self.feedback is not None and time.monotonic()-self.feedback_at < 1
                and time.monotonic()-self.clock_at < 1)

    def request(self, delta):
        if self.active:
            self.node.get_logger().warning('Lift is busy; wait or press SPACE to stop.'); return
        if not self.fresh() or not self.validity.service_is_ready() or not self.pub.get_subscription_count():
            self.node.get_logger().warning('Lift requires live simulation, lift controller and MoveIt.'); return
        if not self.base_stopped or time.monotonic()-self.base_at > 1:
            self.node.get_logger().warning('Stop the base before lifting; press R/F again after it stops.'); return
        q = dict(zip(self.feedback.name, self.feedback.position))
        self.start = q[self.JOINT]
        if not math.isfinite(self.start) or not -.01 <= self.start <= self.UPPER+.01:
            self.node.get_logger().warning('Invalid lift position feedback.'); return
        self.target = min(self.UPPER, max(0., self.start+delta))
        if abs(self.target-self.start) < .002:
            self.node.get_logger().info('Lift is at its travel limit.'); return
        self.active = True; self.finish = 0
        for i in range(11):
            req = GetStateValidity.Request();req.robot_state.is_diff = True
            req.robot_state.joint_state.name = list(q)
            req.robot_state.joint_state.position = [self.start+(self.target-self.start)*i/10 if k==self.JOINT else v for k,v in q.items()]
            self.checks.append(req)
        self.node.get_logger().info(f'Lift {self.start:.3f} -> {self.target:.3f} m; checking path.')

    def stop(self):
        if self.future is not None: self.future.cancel(); self.future = None
        self.checks.clear()
        if self.active and self.fresh():
            value=self.feedback.position[self.feedback.name.index(self.JOINT)]
            msg=JointTrajectory();msg.joint_names=[self.JOINT]
            msg.points=[JointTrajectoryPoint(positions=[value],velocities=[0.],time_from_start=Duration(sec=0,nanosec=500000000))]
            self.pub.publish(msg)
        self.active = False;self.finish = 0

    def tick(self):
        if not self.active: return
        if not self.fresh():
            self.stop();self.node.get_logger().warning('Lift stopped: stale feedback or simulation paused.');return
        if self.finish:
            if self.clock_ns >= self.finish:
                error=abs(self.feedback.position[self.feedback.name.index(self.JOINT)]-self.target)
                self.active=False;self.finish=0
                if error>.015:self.node.get_logger().warning(f'Lift tracking error {error:.3f} m.')
            return
        if self.future is not None:
            if not self.future.done():
                if time.monotonic()-self.request_at>3:self.stop()
                return
            result=self.future.result();self.future=None
            if result is None or not result.valid:
                self.stop();self.node.get_logger().warning('Lift path rejected by MoveIt; no lift motion sent.');return
            if not self.checks:
                duration=max(2.,abs(self.target-self.start)/.025)
                msg=JointTrajectory();msg.joint_names=[self.JOINT]
                msg.points=[JointTrajectoryPoint(positions=[self.start],velocities=[0.],time_from_start=Duration()),
                    JointTrajectoryPoint(positions=[self.target],velocities=[0.],
                        time_from_start=Duration(sec=int(duration),nanosec=round((duration%1)*1e9)))]
                self.pub.publish(msg);self.finish=self.clock_ns+round((duration+.3)*1e9)
                return
        self.future=self.validity.call_async(self.checks.pop(0));self.request_at=time.monotonic()
