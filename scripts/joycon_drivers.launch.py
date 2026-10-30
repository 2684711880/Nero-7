"""Two official NERO drivers for Joy-Con teleoperation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    speed = ParameterValue(LaunchConfiguration('speed_percent'), value_type=int)
    return LaunchDescription([
        DeclareLaunchArgument('speed_percent', default_value='100',
                              choices=[str(i) for i in range(1, 101)]),
        *[Node(package='agx_arm_ctrl', executable='agx_arm_ctrl_single',
               namespace=side, output='screen', parameters=[dict(
                   arm_type='nero', fw_version='v121', can_port=can,
                   effector_type='agx_gripper', auto_enable=False,
                   control_enabled=False, fast_mode=False,
                   speed_percent=speed, pub_rate=200)])
          for side, can in (('left', 'can1'), ('right', 'can0'))],
    ])
