import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from .kinematics import WHEELS, inverse, forward, wrap


class AirController(Node):
    def __init__(self):
        super().__init__('air_controller')
        defaults = {'command_timeout': .5, 'feedback_timeout': .3,
                    'max_linear_speed': .4, 'max_yaw_rate': .8,
                    'linear_acceleration': .4, 'angular_acceleration': .8,
                    'steering_rate': 1.5, 'alignment_tolerance': .12,
                    'alignment_stop_angle': .5}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
            setattr(self, key, self.get_parameter(key).value)
        if not 0 <= self.alignment_tolerance < self.alignment_stop_angle <= math.pi/2:
            raise ValueError('Require 0 <= alignment_tolerance < alignment_stop_angle <= pi/2')
        self.declare_parameter('use_ground_truth_odometry', False)
        self.truth_odom = self.get_parameter('use_ground_truth_odometry').value
        self.truth_received = False
        if self.truth_odom:
            self.create_subscription(ModelStates, '/gazebo/model_states', self.on_truth, 10)
        self.target = np.zeros(3)
        self.filtered = np.zeros(3)
        self.last_cmd = self.last_feedback = -math.inf
        self.angles = [0.] * 4
        self.steering_command = [0.] * 4
        self.have_feedback = False
        self.pose = np.zeros(3)
        self.last_stamp = None
        self.last_tick = None
        self.steer_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(JointState, '/joint_states', self.on_feedback, 10)
        self.create_timer(.02, self.tick)

    def on_truth(self, msg):
        if 'mobile_dual_nero' not in msg.name:
            return
        p = msg.pose[msg.name.index('mobile_dual_nero')]
        q = p.orientation
        self.pose[:] = (p.position.x, p.position.y,
                        math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.truth_received = True

    def on_command(self, msg):
        values = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        if not np.all(np.isfinite(values)):
            self.target[:] = 0.
            self.last_cmd = -math.inf
            return
        norm = np.linalg.norm(values[:2])
        if norm > self.max_linear_speed:
            values[:2] *= self.max_linear_speed / norm
        values[2] = np.clip(values[2], -self.max_yaw_rate, self.max_yaw_rate)
        self.target = values
        self.last_cmd = time.monotonic()

    def on_feedback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        try:
            angles = [positions[w + '_steering_joint'] for w in WHEELS]
            rates = [velocities[w + '_wheel_joint'] for w in WHEELS]
        except KeyError:
            return
        if not all(math.isfinite(x) for x in angles + rates):
            return
        self.angles = angles
        self.last_feedback = time.monotonic()
        if not self.have_feedback:
            self.steering_command = angles.copy()
            self.have_feedback = True
        twist = forward(angles, rates)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and not self.truth_odom:
            dt = stamp - self.last_stamp
            if 0. < dt < .2:
                yaw = self.pose[2] + twist[2] * dt / 2
                self.pose[0] += (math.cos(yaw)*twist[0] - math.sin(yaw)*twist[1]) * dt
                self.pose[1] += (math.sin(yaw)*twist[0] + math.cos(yaw)*twist[1]) * dt
                self.pose[2] += twist[2] * dt
        self.last_stamp = stamp
        if self.truth_odom and not self.truth_received:
            return
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(self.pose[0]), float(self.pose[1])
        odom.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        odom.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = float(twist[0]), float(twist[1])
        odom.twist.twist.angular.z = float(twist[2])
        # Nominal uncertainty, not a calibrated covariance model.
        for i in [0, 7, 35]:
            odom.pose.covariance[i] = .02
            odom.twist.covariance[i] = .02
        for i in [14, 21, 28]:
            odom.pose.covariance[i] = odom.twist.covariance[i] = 1e6
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

    def tick(self):
        stamp = self.get_clock().now().nanoseconds * 1e-9
        dt = .02 if self.last_tick is None else min(.1, max(0., stamp-self.last_tick))
        self.last_tick = stamp
        if not self.have_feedback:
            return
        now = time.monotonic()
        stopped = (now - self.last_cmd > self.command_timeout or
                   now - self.last_feedback > self.feedback_timeout)
        if stopped:
            self.filtered[:] = 0.
            rates = [0.] * 4
        else:
            delta = self.target - self.filtered
            linear_step = self.linear_acceleration * dt
            length = np.linalg.norm(delta[:2])
            if length > linear_step:
                delta[:2] *= linear_step / length
            delta[2] = np.clip(delta[2], -self.angular_acceleration*dt, self.angular_acceleration*dt)
            self.filtered += delta
            desired, rates = inverse(*self.filtered, self.angles)
            step = self.steering_rate * dt
            self.steering_command = [old + np.clip(wrap(new-old), -step, step)
                                     for old, new in zip(self.steering_command, desired)]
            peak_rate = max(abs(rate) for rate in rates)
            # Near an instantaneous rotation center a wheel may need almost
            # zero speed but a fast-changing heading. Weight its angular error
            # by its requested rolling speed so it cannot stop the other three.
            error = (max(abs(wrap(a-b))*abs(rate)/peak_rate
                         for a, b, rate in zip(desired, self.angles, rates))
                     if peak_rate > 1e-6 else 0.)
            # A hard on/off gate at 0.12 rad repeatedly stopped all wheels
            # during continuous translation + yaw. Taper all rates together
            # to preserve their ratios; large misalignment still blocks drive.
            u = np.clip((error-self.alignment_tolerance) /
                        (self.alignment_stop_angle-self.alignment_tolerance), 0., 1.)
            scale = 1. - u*u*(3.-2.*u)
            rates = [rate*scale for rate in rates]
        self.steer_pub.publish(Float64MultiArray(data=[float(x) for x in self.steering_command]))
        self.wheel_pub.publish(Float64MultiArray(data=[float(x) for x in rates]))


def main():
    rclpy.init()
    node = AirController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.wheel_pub.publish(Float64MultiArray(data=[0.] * 4))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
