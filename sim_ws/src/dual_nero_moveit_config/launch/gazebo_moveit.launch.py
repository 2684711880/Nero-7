from pathlib import Path
import os
import socket
import math
import tempfile
import xml.etree.ElementTree as ET
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


def setup(context):
    support_height = float(LaunchConfiguration('support_height').perform(context))
    cabinet_height = float(LaunchConfiguration('cabinet_height').perform(context))
    cabinet_width = float(LaunchConfiguration('cabinet_width').perform(context))
    cabinet_depth = float(LaunchConfiguration('cabinet_depth').perform(context))
    cabinet_front = float(LaunchConfiguration('cabinet_front').perform(context))
    if (not all(math.isfinite(v) for v in (support_height, cabinet_height, cabinet_width,
                                           cabinet_depth, cabinet_front))
            or min(support_height, cabinet_height) < 0
            or min(cabinet_width, cabinet_depth, cabinet_front) <= 0):
        raise ValueError('Scene dimensions must be finite and positive (heights may be zero).')
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
    # Move the fixed world->torso transform, shared by Gazebo, TF and MoveIt.
    robot = ET.fromstring(config.robot_description['robot_description'])
    joint = robot.find("joint[@name='world_joint']")
    origin = joint.find('origin')
    if origin is None:
        origin = ET.SubElement(joint, 'origin')
    origin.set('xyz', f'0 0 {support_height}')
    startup_pose = LaunchConfiguration('startup_arm_pose').perform(context)
    if startup_pose == 'park':
        semantic = ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
        park = semantic.find("group_state[@name='park'][@group='both_arms']")
        for target in park.findall('joint'):
            initial = robot.find(
                f"ros2_control/joint[@name='{target.get('name')}']/state_interface[@name='position']/param[@name='initial_value']")
            initial.text = target.get('value')
    config.robot_description['robot_description'] = ET.tostring(robot, encoding='unicode')
    objects = []
    if support_height > 0:
        # Placeholder for the hardware below the torso, not a chassis dynamics model.
        objects.append(('support_pedestal', (.40, .35, support_height),
                        (0, 0, support_height / 2), '.22 .25 .28 1'))
    if cabinet_height > 0:
        objects.append(('front_cabinet', (cabinet_depth, cabinet_width, cabinet_height),
                        (cabinet_front + cabinet_depth / 2, 0, cabinet_height / 2),
                        '.62 .44 .27 1'))
    world_path = str(share / 'worlds/dual_nero.world')
    if objects:
        world_root = ET.parse(world_path).getroot()
        world = world_root.find('world')
        world.set('name', 'nero_front_cabinet')
        gui = ET.SubElement(world, 'gui')
        camera = ET.SubElement(gui, 'camera', name='user_camera')
        ET.SubElement(camera, 'pose').text = '2.8 -3.2 2.2 0 .36 2.35'
        for name, size, xyz, color in objects:
            model = ET.SubElement(world, 'model', name=name)
            ET.SubElement(model, 'static').text = 'true'
            ET.SubElement(model, 'pose').text = ' '.join(map(str, (*xyz, 0, 0, 0)))
            body = ET.SubElement(model, 'link', name='body')
            for tag in ('collision', 'visual'):
                item = ET.SubElement(body, tag, name=tag)
                ET.SubElement(ET.SubElement(ET.SubElement(item, 'geometry'), 'box'), 'size').text = ' '.join(map(str, size))
                if tag == 'visual':
                    material = ET.SubElement(item, 'material')
                    for key in ('ambient', 'diffuse'):
                        ET.SubElement(material, key).text = color
        with tempfile.NamedTemporaryFile(mode='w', suffix='.world', prefix='nero-cabinet-', delete=False) as f:
            f.write(ET.tostring(world_root, encoding='unicode'))
            world_path = f.name
    rows = [' '.join(map(str, (name, 'box', *size, *xyz)))
            for name, size, xyz, color in objects]
    sim = {'use_sim_time': True}
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'dual_nero', '-topic', 'robot_description'])
    controllers = [Node(package='controller_manager', executable='spawner', output='screen',
                        arguments=[name, '--controller-manager', '/controller_manager',
                                   '--controller-manager-timeout', '90'])
                   for name in ['joint_state_broadcaster', 'left_arm_controller',
                                'right_arm_controller', 'left_gripper_controller',
                                'right_gripper_controller']]
    ground = Node(package='dual_nero_moveit_config', executable='add_ground', output='screen',
                  parameters=[{'environment_objects': rows}] if rows else [])
    rviz = Node(package='rviz2', executable='rviz2', output='log',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', str(share / 'config/moveit.rviz')],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines,
                         config.joint_limits, sim])
    return [
        # Gazebo's default materials are needed even when only the model package is sourced.
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', str(share.parent)),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', str(share.parent)),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo_share / 'launch/gazebo.launch.py')),
                                 launch_arguments={'world': world_path,
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
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('startup_arm_pose', default_value='home', choices=['home', 'park']),
        DeclareLaunchArgument('support_height', default_value='0.0',
                              description='Torso base_link height above ground, metres'),
        DeclareLaunchArgument('cabinet_height', default_value='0.0',
                              description='Cabinet top height; zero disables the cabinet'),
        DeclareLaunchArgument('cabinet_width', default_value='1.2'),
        DeclareLaunchArgument('cabinet_depth', default_value='0.4'),
        DeclareLaunchArgument('cabinet_front', default_value='0.50',
                              description='Near face X from the torso origin, metres'),
        OpaqueFunction(function=check_gazebo_master),
        OpaqueFunction(function=setup),
    ])
