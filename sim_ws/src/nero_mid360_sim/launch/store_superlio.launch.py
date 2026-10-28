"""One simulation-only entry. The physical Livox driver is deliberately absent."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def robot_view(context):
    from mobile_dual_nero.model import compose
    from mobile_dual_nero.mid360 import mount_pose, moves_with_upper_body
    values = {key: LaunchConfiguration(key).perform(context)
              for key in ('mid360_mount', 'mid360_xyz', 'mid360_rpy', 'lift_height', 'mid360_tilt_deg')}
    description, _ = compose(**values)
    xyz, rpy = mount_pose(values['mid360_mount'], values['mid360_xyz'], values['mid360_rpy'], values['mid360_tilt_deg'])
    remaps = [('/tf', '/lio/tf'), ('/tf_static', '/lio/tf_static')]
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='lio_robot_state_publisher',
             parameters=[{'robot_description': description, 'frame_prefix': 'lio_robot/',
                          'use_sim_time': True}],
             remappings=[*remaps, ('robot_description', '/lio/robot_description')], output='log'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='raw_mid360_display_frame',
             arguments=['--frame-id', 'imu', '--child-frame-id', 'mid360_link'],
             parameters=[{'use_sim_time': True}], remappings=remaps, output='log'),
        Node(package='nero_mid360_sim', executable='mapping_robot_pose',
             parameters=[{'mount_xyz': xyz, 'mount_rpy': rpy, 'moving_mount': moves_with_upper_body(values['mid360_mount']), 'use_sim_time': True}],
             remappings=remaps, output='log'),
    ]


def algorithm_nodes(context):
    import math
    share = Path(get_package_share_directory('nero_mid360_sim'))
    algorithm = LaunchConfiguration('algorithm').perform(context)
    text = LaunchConfiguration('lio_blind').perform(context).strip()
    self_filter = LaunchConfiguration('self_filter').perform(context) == 'true'
    blind = float(text) if text else (0.15 if self_filter else 2.0 if algorithm == 'superlio' else 0.3)
    if not math.isfinite(blind) or blind < 0:
        raise ValueError('lio_blind must be a finite, non-negative distance in metres')
    info = LogInfo(msg=f'Mapping algorithm={algorithm}, near-range rejection={blind:g} m (raw sensor unchanged)')
    lidar_topic = '/mid360/lidar_filtered' if self_filter else '/mid360/lidar'
    filter_nodes = []
    if self_filter:
        from mobile_dual_nero.model import compose
        values = {key: LaunchConfiguration(key).perform(context)
                  for key in ('mid360_mount', 'mid360_xyz', 'mid360_rpy', 'lift_height', 'mid360_tilt_deg')}
        description, _ = compose(**values)
        padding = float(LaunchConfiguration('self_filter_padding').perform(context))
        if not math.isfinite(padding) or not 0 <= padding <= .10:
            raise ValueError('self_filter_padding must be in [0, 0.10] metres')
        filter_nodes.append(Node(package='nero_mid360_sim', executable='body_self_filter',
            parameters=[{'use_sim_time': True, 'robot_description': description, 'padding': padding}], output='log'))
    if algorithm == 'superlio':
        return [info, *filter_nodes, Node(package='super_lio', executable='super_lio_node', name='nero_super_lio',
                    parameters=[str(share/'config/superlio_sim.yaml'), {'lio.sensor.blind': blind, 'lio.ros.lidar_topic': lidar_topic}],
                    remappings=[('/tf', '/lio/tf'), ('/tf_static', '/lio/tf_static')], output='log')]
    get_package_share_directory('fast_lio')  # resolve before starting the simulation
    return [info, *filter_nodes,
        Node(package='fast_lio', executable='fastlio_mapping', name='nero_fastlio2',
             parameters=[str(share/'config/fastlio_sim.yaml'),
                         {'use_sim_time': True, 'preprocess.blind': blind, 'common.lid_topic': lidar_topic}],
             remappings=[('/Odometry','/fastlio/odom'), ('/cloud_registered','/fastlio/cloud_world'),
                         ('/path','/fastlio/path'), ('/tf','/fastlio/tf'),
                         ('/tf_static','/fastlio/tf_static')], output='log'),
        Node(package='nero_mid360_sim', executable='fastlio_output_adapter',
             parameters=[{'use_sim_time': True}],
             remappings=[('/tf','/lio/tf'),('/tf_static','/lio/tf_static')], output='log'),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('nero_mid360_sim'))
    mobile = Path(get_package_share_directory('mobile_dual_nero'))
    args = [DeclareLaunchArgument('algorithm', default_value='superlio', choices=['superlio', 'fastlio2']),
            DeclareLaunchArgument('mid360_mount', default_value='chassis_up', choices=['chassis_up', 'chest_down', 'head_tilt']),
            DeclareLaunchArgument('mid360_xyz', default_value=''),
            DeclareLaunchArgument('mid360_rpy', default_value=''),
            DeclareLaunchArgument('mid360_tilt_deg', default_value='45', description='Head pitch in degrees; positive tilts toward robot front'),
            DeclareLaunchArgument('mid360_points_per_second', default_value='128000'),
            DeclareLaunchArgument('ceiling', default_value='true', choices=['true','false']),
            DeclareLaunchArgument('lio_blind', default_value='', description='Metres; empty uses 0.15 with self_filter, otherwise Super-LIO 2.0 or FAST-LIO2 0.3'),
            DeclareLaunchArgument('self_filter', default_value='false', choices=['true','false']),
            DeclareLaunchArgument('self_filter_padding', default_value='0.025', description='Body box padding in metres, 0..0.10'),
            DeclareLaunchArgument('lift_height', default_value='0.0'),
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='true')]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(mobile / 'launch/gazebo_moveit.launch.py')),
        launch_arguments={
            'environment': 'convenience_store', 'use_rviz': 'false',
            'command_bridge': 'false', 'startup_arm_pose': 'relaxed',
            **{key: LaunchConfiguration(key) for key in ('mid360_mount', 'mid360_xyz', 'mid360_rpy', 'mid360_points_per_second', 'ceiling', 'gui', 'lift_height', 'mid360_tilt_deg')}
        }.items())
    rviz = Node(package='rviz2', executable='rviz2', name='nero_mapping_view',
                parameters=[{'use_sim_time': True}],
                arguments=['-d', str(share/'config/mapping.rviz')],
                condition=IfCondition(LaunchConfiguration('rviz')),
                remappings=[('/tf', '/lio/tf'), ('/tf_static', '/lio/tf_static')], output='log')
    return LaunchDescription([*args, OpaqueFunction(function=algorithm_nodes), sim, OpaqueFunction(function=robot_view), rviz])
