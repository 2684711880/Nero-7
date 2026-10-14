from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_dual_nero.model import compose
from mobile_dual_nero.planning import planning_config


def setup(context):
    config = planning_config(*compose(LaunchConfiguration('mount_xyz').perform(context),
                                      LaunchConfiguration('mount_rpy').perform(context)))
    return [Node(package='dual_nero_moveit_config', executable='verify_dual', output='screen',
                 parameters=[config.robot_description, config.robot_description_semantic,
                             config.robot_description_kinematics, {'use_sim_time': True}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0.275'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        OpaqueFunction(function=setup),
    ])
