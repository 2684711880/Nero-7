from pathlib import Path
import os
import socket
from urllib.parse import urlparse
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            AppendEnvironmentVariable, SetEnvironmentVariable,
                            OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ranger_air_gazebo.model import simulation_description


def check_master(context):
    uri = urlparse(os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345'))
    try:
        with socket.create_connection((uri.hostname or '127.0.0.1', uri.port or 11345), timeout=.5):
            pass
    except OSError:
        return []
    raise RuntimeError('Gazebo is already running. Stop the previous simulation with Ctrl-C first.')


def generate_launch_description():
    share = Path(get_package_share_directory('ranger_air_gazebo'))
    model_share = Path(get_package_share_directory('ranger_air_description'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    description = simulation_description()
    sim = {'use_sim_time': True}
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'ranger_air', '-topic', 'robot_description', '-z', '0.03'])
    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'steering_controller', 'wheel_controller',
                                  '--controller-manager-timeout', '60'])
    drive = Node(package='ranger_air_gazebo', executable='air_controller',
                 parameters=[sim], output='screen')

    def after_spawn(event, context):
        return [controllers] if event.returncode == 0 else [Shutdown(reason='AIR spawn failed')]

    def after_controllers(event, context):
        return [drive] if event.returncode == 0 else [Shutdown(reason='AIR controller activation failed')]

    return LaunchDescription([
        OpaqueFunction(function=check_master),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', str(model_share.parent)),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo_share / 'launch/gazebo.launch.py')),
                                 launch_arguments={'world': str(share / 'worlds/control.world'),
                                                   'gui': LaunchConfiguration('gui')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}, sim], output='screen'),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=after_controllers)),
        RegisterEventHandler(OnProcessExit(target_action=drive,
                             on_exit=[Shutdown(reason='AIR drive node exited')])),
        Node(package='rviz2', executable='rviz2', output='log', parameters=[sim],
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', str(share / 'config/control.rviz')]),
    ])
