"""Map joint names; forward commands only for the controller currently executing."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult


def renamed(message, side, to_model, allowed=None):
    result = JointState()
    result.header = message.header
    result.header.frame_id = ''
    prefix = side + '_'
    for index, name in enumerate(message.name):
        short = name if to_model else name.removeprefix(prefix)
        if not to_model and not name.startswith(prefix):
            continue
        if short not in [f'joint{i}' for i in range(1, 8)] + ['gripper']:
            continue
        if allowed is not None and short not in allowed:
            continue
        if index >= len(message.position) or not math.isfinite(message.position[index]):
            continue
        result.name.append(prefix+short if to_model else short)
        result.position.append(message.position[index])
    return result


class JointBridge(Node):
    def __init__(self):
        super().__init__('dual_joint_bridge')
        self.feedback = self.create_publisher(JointState, '/joint_states', 10)
        for side in ('left', 'right'):
            self.declare_parameter(side+'_gripper_force', 1.0)
        self.add_on_set_parameters_callback(self.validate_force)
        self.outputs = {}
        self.active = {(side, part): False for side in ('left', 'right') for part in ('arm', 'gripper')}
        for side in ('left', 'right'):
            self.outputs[side] = self.create_publisher(JointState, f'/{side}/control/joint_states', 10)
            self.create_subscription(JointState, f'/{side}/feedback/joint_states',
                lambda msg, s=side: self.feedback.publish(renamed(msg, s, True)), qos_profile_sensor_data)
            for part in ('arm', 'gripper'):
                self.create_subscription(GoalStatusArray, f'/{side}_{part}_controller/follow_joint_trajectory/_action/status',
                    lambda msg, s=side, p=part: self.status(msg, s, p), 10)
        self.create_subscription(JointState, '/dual_control/joint_states', self.commands, 10)

    @staticmethod
    def validate_force(parameters):
        for parameter in parameters:
            if parameter.name in ('left_gripper_force', 'right_gripper_force'):
                value = parameter.value
                if type(value) is not float or not math.isfinite(value) or not .5 <= value <= 3.:
                    return SetParametersResult(successful=False, reason='gripper force must be 0.5..3.0 N')
        return SetParametersResult(successful=True)

    def status(self, message, side, part):
        # Same accepted/executing/canceling definition as the official control gate.
        self.active[side, part] = any(item.status in (1, 2, 3) for item in message.status_list)

    def commands(self, message):
        for side in ('left', 'right'):
            allowed = set()
            if self.active[side, 'arm']:
                allowed.update(f'joint{i}' for i in range(1, 8))
            if self.active[side, 'gripper']:
                allowed.add('gripper')
            command = renamed(message, side, False, allowed)
            arm = [name for name in command.name if name.startswith('joint')]
            if arm and len(arm) != 7:
                continue  # Official callback fills omitted arm joints with zero: send complete arms only.
            if command.name:
                command.effort = [self.get_parameter(side+'_gripper_force').value
                                  if name == 'gripper' else 0.0 for name in command.name]
                self.outputs[side].publish(command)


def main():
    rclpy.init()
    node = JointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
