from pathlib import Path
import os
import socket
from urllib.parse import urlparse
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, AppendEnvironmentVariable,
                            SetEnvironmentVariable, OpaqueFunction, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def check_gazebo_master(context):
    uri = urlparse(os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345'))
    try:
        with socket.create_connection((uri.hostname or '127.0.0.1', uri.port or 11345), timeout=0.5):
            pass
    except OSError:
        return []
    raise RuntimeError('Gazebo master is already running. Stop the previous simulation with Ctrl-C before launching another dual NERO instance.')


def generate_launch_description():
    share = Path(get_package_share_directory('dual_nero_moveit_config'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    config = (MoveItConfigsBuilder('dual_nero', package_name='dual_nero_moveit_config')
              .robot_description(file_path='urdf/dual_nero.urdf.xacro')
              .robot_description_semantic(file_path='config/dual_nero.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .planning_pipelines(pipelines=['ompl'])
              .to_moveit_configs())
    sim = {'use_sim_time': True}
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'dual_nero', '-topic', 'robot_description'])
    controllers = [Node(package='controller_manager', executable='spawner', output='screen',
                        arguments=[name, '--controller-manager', '/controller_manager',
                                   '--controller-manager-timeout', '90'])
                   for name in ['joint_state_broadcaster', 'left_arm_controller',
                                'right_arm_controller', 'left_gripper_controller',
                                'right_gripper_controller']]
    ground = Node(package='dual_nero_moveit_config', executable='add_ground', output='screen')
    rviz = Node(package='rviz2', executable='rviz2', output='log',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', str(share / 'config/moveit.rviz')],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines,
                         config.joint_limits, sim])
    return LaunchDescription([
        OpaqueFunction(function=check_gazebo_master),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # Gazebo's default materials are needed even when only the model package is sourced.
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', str(share.parent)),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', str(share.parent)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo_share / 'launch/gazebo.launch.py')),
                                 launch_arguments={'world': str(share / 'worlds/dual_nero.world'),
                                                   'gui': LaunchConfiguration('gui')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[config.robot_description, sim]),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=controllers)),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[config.to_dict(), sim,
                         {'publish_robot_description_semantic': True,
                          'publish_planning_scene': True,
                          'publish_geometry_updates': True,
                          'publish_state_updates': True,
                          'publish_transforms_updates': True}]),
        ground,
        RegisterEventHandler(OnProcessExit(
            target_action=ground,
            on_exit=lambda event, context: [rviz] if event.returncode == 0 else [
                Shutdown(reason='MoveIt ground initialization failed; refusing to start unprotected planning UI.')])),
    ])
