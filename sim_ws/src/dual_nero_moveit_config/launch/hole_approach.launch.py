from pathlib import Path
import os
import socket
import math
import tempfile
import json
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from hole_scene import add_hole_scene
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
    if not LaunchConfiguration('camera_calibration').perform(context):
        raise ValueError('Pass camera_calibration:=/path/to/config/handeye/right/d405.json or use scripts/sim_hole.sh start')
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
    elif startup_pose in ('hole_ready', 'before_observe', 'interactive'):
        values = [-1.7594892, 1.7249772, .1795630, .6631126, 1.0165273, .1185619, -.0919321,
                  .1076393, -.9554579, 1.4951906, 1.6442938, .2765393, -.1913224, .3182951]
        board_j1_offset = float(LaunchConfiguration('board_j1_offset').perform(context))
        if not math.isfinite(board_j1_offset) or abs(board_j1_offset) > .10:
            raise ValueError('board_j1_offset must be finite and within ±0.10 rad')
        values[0] += board_j1_offset
        if startup_pose == 'before_observe':
            values[7] += .55
        for name, value in zip([f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)], values):
            robot.find(f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param[@name='initial_value']").text = str(value)
        if startup_pose == 'interactive':
            semantic = ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
            park = semantic.find("group_state[@name='park'][@group='right_arm']")
            right = {item.get('name'): float(item.get('value')) for item in park.findall('joint')}
            feedback_path = LaunchConfiguration('right_feedback_file').perform(context)
            if feedback_path:
                supplied = json.loads(Path(feedback_path).read_text())
                if set(supplied) != set(right) or not all(math.isfinite(float(v)) for v in supplied.values()):
                    raise ValueError('Right feedback must contain seven finite named joint positions')
                for name, value in supplied.items():
                    limit = robot.find(f"joint[@name='{name}']/limit")
                    if not float(limit.get('lower')) <= float(value) <= float(limit.get('upper')):
                        raise ValueError(f'Right feedback outside model limits: {name}')
                right = supplied
            for name, value in right.items():
                robot.find(f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param[@name='initial_value']").text = str(value)
            print('Right startup source:', 'passive ROS feedback' if feedback_path else 'park', flush=True)
    semantic = ET.fromstring(config.robot_description_semantic['robot_description_semantic'])
    calibration_path = Path(LaunchConfiguration('camera_calibration').perform(context))
    add_hole_scene(robot, semantic, json.loads(calibration_path.read_text()))
    # Leave a 0.5 mm clearance on each side of the 8 mm thick fixed plate.
    for name, value in [('left_gripper_joint1', .0045), ('left_gripper_joint2', -.0045)]:
        robot.find(f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param[@name='initial_value']").text = str(value)
    config.robot_description_semantic['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
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
        DeclareLaunchArgument('camera_calibration', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('startup_arm_pose', default_value='interactive', choices=['home', 'park', 'hole_ready', 'before_observe', 'interactive']),
        DeclareLaunchArgument('right_feedback_file', default_value=''),
        DeclareLaunchArgument('board_j1_offset', default_value='0.0', description='Simulation left J1 starting offset (rad)'),
        DeclareLaunchArgument('support_height', default_value='0.28',
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
