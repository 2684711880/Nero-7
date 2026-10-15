from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_dual_nero.model import compose
from mobile_dual_nero.planning import planning_config


def setup(context):
    config = planning_config(*compose(LaunchConfiguration('mount_xyz').perform(context),
                                      LaunchConfiguration('mount_rpy').perform(context)))
    share = Path(get_package_share_directory('mobile_dual_nero'))
    return [Node(package='rviz2', executable='rviz2', output='log',
                 arguments=['-d', str(share/'config/moveit.rviz')],
                 parameters=[config.to_dict(), {'use_sim_time': True}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0.275'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        OpaqueFunction(function=setup),
    ])
