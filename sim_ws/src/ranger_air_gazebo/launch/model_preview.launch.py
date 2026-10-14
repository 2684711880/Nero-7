from pathlib import Path
import os
import socket
import xml.etree.ElementTree as ET
from urllib.parse import urlparse
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_gazebo_master(context):
    uri = urlparse(os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345'))
    try:
        with socket.create_connection((uri.hostname or '127.0.0.1', uri.port or 11345), timeout=0.5):
            pass
    except OSError:
        return []
    raise RuntimeError('Gazebo is already running. Stop the previous simulation before opening AIR preview.')


def generate_launch_description():
    model_share = Path(get_package_share_directory('ranger_air_description'))
    share = Path(get_package_share_directory('ranger_air_gazebo'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    # spawn_entity uses lxml on a Unicode string, so omit the XML encoding declaration.
    description = ET.tostring(ET.parse(model_share / 'urdf/ranger_air_description.urdf').getroot(), encoding='unicode')
    return LaunchDescription([
        OpaqueFunction(function=check_gazebo_master),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', str(model_share.parent)),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_share / 'launch/gazebo.launch.py')),
            launch_arguments={'world': str(share / 'worlds/preview.world'),
                              'pause': 'true', 'gui': LaunchConfiguration('gui')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             # This paused preview uses wall time for the synthetic joint states and TF.
             parameters=[{'robot_description': description, 'use_sim_time': False}],
             output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='preview_joint_state_publisher',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             parameters=[{'use_sim_time': False}], output='log'),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             parameters=[{'use_sim_time': False}],
             arguments=['-d', str(model_share / 'rviz/display.rviz')], output='log'),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'ranger_air', '-topic', 'robot_description', '-z', '0.03']),
    ])
