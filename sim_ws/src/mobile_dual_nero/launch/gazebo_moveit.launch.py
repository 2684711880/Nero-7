from pathlib import Path
import tempfile
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
from ranger_air_gazebo.model import contact_spawn_pose
from mobile_dual_nero.planning import planning_config
from mobile_dual_nero.convenience_store import convenience_store_objects
from mobile_dual_nero.environment import warehouse_objects, collision_rows, world_xml


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
                                    LaunchConfiguration('mount_rpy').perform(context),
                                    LaunchConfiguration('startup_arm_pose').perform(context),
                                    mid360_mount=LaunchConfiguration('mid360_mount').perform(context),
                                    mid360_xyz=LaunchConfiguration('mid360_xyz').perform(context),
                                    mid360_rpy=LaunchConfiguration('mid360_rpy').perform(context),
                                    mid360_tilt_deg=LaunchConfiguration('mid360_tilt_deg').perform(context),
                                    mid360_points_per_second=int(LaunchConfiguration('mid360_points_per_second').perform(context)),
                                    lift_height=float(LaunchConfiguration('lift_height').perform(context)))
    config = planning_config(description, semantic)
    sim = {'use_sim_time': True}
    environment = LaunchConfiguration('environment').perform(context)
    objects = {'warehouse': warehouse_objects,
               'convenience_store': lambda: convenience_store_objects(
                   ceiling=LaunchConfiguration('ceiling').perform(context).lower() == 'true'),
               'empty': lambda: []}[environment]()
    has_bottle = any(obj['name'] == 'water_bottle' for obj in objects)
    world = str(air / 'worlds/control.world')
    if objects:
        # Unique launch-local file, generated from the same geometry sent to MoveIt.
        with tempfile.NamedTemporaryFile(mode='w', suffix='.world', prefix=f'nero-{environment}-', delete=False) as f:
            xml = world_xml(world, objects, scene=environment)
            if has_bottle and LaunchConfiguration('sim_grasp').perform(context) == 'true':
                xml = xml.replace('</world>', '<plugin name="bottle_grasp" filename="libbottle_grasp.so"/></world>')
            f.write(xml)
            world = f.name
    # Match the existing tire geometry before free settling, without changing axle locations.
    spawn_z, spawn_roll, spawn_pitch = contact_spawn_pose(description)
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'mobile_dual_nero', '-topic', 'robot_description', '-z', str(spawn_z),
                            '-R', str(spawn_roll), '-P', str(spawn_pitch)])
    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['upper_body_lift_controller', 'joint_state_broadcaster', 'steering_controller', 'wheel_controller',
                                  'left_arm_controller', 'right_arm_controller',
                                  'left_gripper_controller', 'right_gripper_controller',
                                  '--controller-manager-timeout', '90'])
    drive = Node(package='ranger_air_gazebo', executable='air_controller', output='screen',
                 parameters=[sim, {'use_ground_truth_odometry': LaunchConfiguration('ground_truth_odometry'), 'max_linear_speed': .2, 'max_yaw_rate': .4,
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
    ground = Node(package='mobile_dual_nero', executable='add_ground', output='screen',
                  parameters=[{'environment_objects': collision_rows(objects)}] if objects else [])
    sync = Node(package='mobile_dual_nero', executable='sync_bottle', output='screen',
                parameters=[sim], condition=IfCondition(str(has_bottle).lower()))
    command_bridge = Node(package='mobile_dual_nero', executable='official_command_bridge',
                          output='screen', parameters=[config.to_dict(), sim],
                          condition=IfCondition(LaunchConfiguration('command_bridge')))
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                parameters=[config.robot_description, config.robot_description_semantic,
                            config.robot_description_kinematics, config.planning_pipelines,
                            config.joint_limits, sim],
                arguments=['-d', str(share / 'config/moveit.rviz')])
    sensor_paths = []
    if LaunchConfiguration('mid360_mount').perform(context) != 'none':
        sensor_share = Path(get_package_share_directory('nero_mid360_sim'))
        sensor_paths = [AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', str(sensor_share.parent.parent / 'lib'))]
    return [
        *sensor_paths,
        # Gazebo converts package:// mesh URIs to model://; each isolated colcon
        # prefix needs its own share directory in the client's search path.
        *[AppendEnvironmentVariable('GAZEBO_MODEL_PATH', path) for path in gazebo_model_paths()],
        AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', str(share.parent.parent / 'lib')),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', str(share.parent)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch/gazebo.launch.py')),
                                 launch_arguments={'world': world,
                                                   'gui': LaunchConfiguration('gui')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[config.robot_description, sim], output='screen'),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=lambda event, ctx: [controllers] if event.returncode == 0 else [Shutdown(reason='Spawn failed')])),
        RegisterEventHandler(OnProcessExit(target_action=controllers,
            on_exit=lambda event, ctx: [drive, move_group, ground] if event.returncode == 0 else [Shutdown(reason='Controller activation failed')])),
        RegisterEventHandler(OnProcessExit(target_action=ground,
            on_exit=lambda event, ctx: [sync, rviz, command_bridge] if event.returncode == 0 else [Shutdown(reason='MoveIt ground initialization failed')])),
        RegisterEventHandler(OnProcessExit(target_action=drive, on_exit=[Shutdown(reason='Drive node exited')])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('sim_grasp', default_value='false'),
        DeclareLaunchArgument('ground_truth_odometry', default_value='false'),
        DeclareLaunchArgument('environment', default_value='warehouse', choices=['warehouse', 'convenience_store', 'empty']),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('command_bridge', default_value='true'),
        DeclareLaunchArgument('startup_arm_pose', default_value='relaxed', choices=['relaxed', 'home']),
        DeclareLaunchArgument('ceiling', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('mid360_mount', default_value='none', choices=['none', 'chassis_up', 'chest_down', 'head_tilt']),
        DeclareLaunchArgument('mid360_xyz', default_value=''),
        DeclareLaunchArgument('mid360_rpy', default_value=''),
        DeclareLaunchArgument('mid360_tilt_deg', default_value='45', description='Head mount pitch in degrees, -90..90'),
        DeclareLaunchArgument('mid360_points_per_second', default_value='128000'),
        DeclareLaunchArgument('lift_height', default_value='0.0', description='Initial upper-body lift height, 0..0.8 metres'),
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0.275'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        OpaqueFunction(function=setup),
    ])
