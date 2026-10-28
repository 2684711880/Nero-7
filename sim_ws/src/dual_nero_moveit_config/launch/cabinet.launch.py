"""Fixed dual NERO torso with a cabinet in front; all dimensions in metres."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    defaults = {'support_height': '0.28', 'cabinet_height': '0.60',
                'cabinet_width': '1.20', 'cabinet_depth': '0.40',
                'cabinet_front': '0.50', 'gui': 'true', 'use_rviz': 'true',
                'startup_arm_pose': 'park'}
    share = Path(get_package_share_directory('dual_nero_moveit_config'))
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/gazebo_moveit.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
