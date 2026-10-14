from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, AppendEnvironmentVariable, SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mobile_dual_nero.model import compose, gazebo_model_paths
from mobile_dual_nero.planning import planning_config


def setup(context):
    # Same guard as the standalone chassis; never spawn into another running world.
    import socket
    import os
    from urllib.parse import urlparse
    uri = urlparse(os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345'))
    try:
        with socket.create_connection((uri.hostname or '127.0.0.1', uri.port or 11345), timeout=.5):
            pass
    except OSError:
        pass
    else:
        raise RuntimeError('Gazebo is already running. Stop the previous simulation first.')
    share = Path(get_package_share_directory('mobile_dual_nero'))
    air = Path(get_package_share_directory('ranger_air_gazebo'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    description, semantic = compose(LaunchConfiguration('mount_xyz').perform(context),
                                    LaunchConfiguration('mount_rpy').perform(context))
    config = planning_config(description, semantic)
    sim = {'use_sim_time': True}
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'mobile_dual_nero', '-topic', 'robot_description', '-z', '.03'])
    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'steering_controller', 'wheel_controller',
                                  'left_arm_controller', 'right_arm_controller',
                                  'left_gripper_controller', 'right_gripper_controller',
                                  '--controller-manager-timeout', '90'])
    drive = Node(package='ranger_air_gazebo', executable='air_controller', output='screen',
                 parameters=[sim, {'max_linear_speed': .2, 'max_yaw_rate': .4,
                                   'linear_acceleration': .15, 'angular_acceleration': .3,
                                   'steering_rate': .8}])
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[config.to_dict(), sim,
                                  {'publish_robot_description': True,
                                   'publish_robot_description_semantic': True,
                                   'publish_planning_scene': True,
                                   'publish_geometry_updates': True,
                                   'publish_state_updates': True,
                                   'publish_transforms_updates': True}])
    ground = Node(package='mobile_dual_nero', executable='add_ground', output='screen')
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                parameters=[config.robot_description, config.robot_description_semantic,
                            config.robot_description_kinematics, config.planning_pipelines,
                            config.joint_limits, sim],
                arguments=['-d', str(share / 'config/moveit.rviz')])
    return [
        # Gazebo converts package:// mesh URIs to model://; each isolated colcon
        # prefix needs its own share directory in the client's search path.
        *[AppendEnvironmentVariable('GAZEBO_MODEL_PATH', path) for path in gazebo_model_paths()],
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', str(share.parent)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch/gazebo.launch.py')),
                                 launch_arguments={'world': str(air / 'worlds/control.world'),
                                                   'gui': LaunchConfiguration('gui')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[config.robot_description, sim], output='screen'),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=lambda event, ctx: [controllers] if event.returncode == 0 else [Shutdown(reason='Spawn failed')])),
        RegisterEventHandler(OnProcessExit(target_action=controllers,
            on_exit=lambda event, ctx: [drive, move_group, ground] if event.returncode == 0 else [Shutdown(reason='Controller activation failed')])),
        RegisterEventHandler(OnProcessExit(target_action=ground,
            on_exit=lambda event, ctx: [rviz] if event.returncode == 0 else [Shutdown(reason='MoveIt ground initialization failed')])),
        RegisterEventHandler(OnProcessExit(target_action=drive, on_exit=[Shutdown(reason='Drive node exited')])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0.275'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        OpaqueFunction(function=setup),
    ])
