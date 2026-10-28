"""Compose the installed official single-arm profile with official Open Nero mounts."""
import copy
import math
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext


def compose(support_height=0.0):
    support_height = float(support_height)
    if not math.isfinite(support_height) or support_height < 0:
        raise ValueError('support_height must be a finite nonnegative number in metres')
    official = Path(get_package_share_directory('agx_arm_moveit'))
    sys.path.insert(0, str(official / 'launch'))
    spec = importlib.util.spec_from_file_location('nero_official_builder', official / 'launch/_moveit_config_builder.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    context = LaunchContext()
    context.launch_configurations.update(arm_type='nero', effector_type='agx_gripper',
        revo2_type='left', tcp_offset='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]')
    cfg = builder.build_moveit_config(context)
    assets = Path(get_package_share_directory('nero_dual_bringup')) / 'open_nero_description'
    fixture = ET.parse(assets / 'urdf/open_nero_description.urdf').getroot()
    mounts = ET.parse(assets / 'urdf/dual_nero.xacro').getroot()
    robot = ET.Element('robot', name='dual_nero')
    # Official torso; cameras are absent on the current hardware.
    base = copy.deepcopy(fixture.find("link[@name='base_link']"))
    for mesh in base.findall('.//mesh'):
        mesh.set('filename', mesh.get('filename').replace(
            'package://open_nero_description/', 'package://nero_dual_bringup/open_nero_description/'))
    robot.append(base)
    ET.SubElement(robot, 'link', name='world')
    world_joint = ET.SubElement(robot, 'joint', name='world_joint', type='fixed')
    ET.SubElement(world_joint, 'parent', link='world')
    ET.SubElement(world_joint, 'child', link='base_link')
    ET.SubElement(world_joint, 'origin', xyz=f'0 0 {support_height}', rpy='0 0 0')
    semantic = ET.Element('robot', name='dual_nero')
    controllers = {'update_rate': 200,
        'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'}}
    controller_config = {}
    limits = {}
    kinematics = {}
    moveit_controllers = {'controller_names': []}
    for side in ('left', 'right'):
        prefix = side + '_'
        single = ET.fromstring(cfg.robot_description['robot_description'])
        for child in list(single):
            if child.tag == 'link' and child.get('name') == 'world':
                continue
            if child.tag == 'joint' and child.find("parent[@link='world']") is not None:
                continue
            for element in child.iter():
                if element.tag in ('link', 'joint', 'ros2_control') and 'name' in element.attrib:
                    element.set('name', prefix + element.get('name'))
                for key in ('link', 'joint'):
                    if key in element.attrib:
                        element.set(key, prefix + element.get(key))
            robot.append(child)
        mount = copy.deepcopy(mounts.find(f"joint[@name='{side}_arm_joint']"))
        mount.find('child').set('link', prefix + 'base_link')
        robot.append(mount)
        srdf = ET.fromstring(cfg.robot_description_semantic['robot_description_semantic'])
        for child in list(srdf):
            if child.tag == 'virtual_joint':
                continue
            for element in child.iter():
                if element.tag in ('group', 'joint', 'passive_joint', 'end_effector') and 'name' in element.attrib:
                    element.set('name', prefix + element.get('name'))
                if element.tag == 'link' and 'name' in element.attrib:
                    element.set('name', prefix + element.get('name'))
                for key in ('base_link', 'tip_link', 'link1', 'link2', 'parent_link', 'group', 'parent_group'):
                    if key in element.attrib:
                        element.set(key, prefix + element.get(key))
            semantic.append(child)
        ET.SubElement(semantic, 'disable_collisions', link1='base_link', link2=prefix+'base_link', reason='Adjacent')
        for group in ('arm', 'gripper'):
            name = prefix + group + '_controller'
            joints = [prefix+f'joint{i}' for i in range(1, 8)] if group == 'arm' else [prefix+'gripper']
            controllers[name] = {'type': 'joint_trajectory_controller/JointTrajectoryController'}
            controller_config[name] = {'ros__parameters': {'joints': joints,
                'command_interfaces': ['position'], 'state_interfaces': ['position', 'velocity']}}
            moveit_controllers['controller_names'].append(name)
            moveit_controllers[name] = {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                'default': True, 'joints': joints}
        source_limits = cfg.joint_limits['robot_description_planning']['joint_limits']
        limits.update({prefix+k: copy.deepcopy(v) for k, v in source_limits.items()})
        kinematics[prefix+'arm'] = copy.deepcopy(cfg.robot_description_kinematics['robot_description_kinematics']['arm'])
    group = ET.SubElement(semantic, 'group', name='both_arms')
    ET.SubElement(group, 'group', name='left_arm')
    ET.SubElement(group, 'group', name='right_arm')
    park = ET.SubElement(semantic, 'group_state', name='park', group='both_arms')
    for side in ('left', 'right'):
        arm_park = semantic.find(f"group_state[@name='park'][@group='{side}_arm']")
        if arm_park is None:
            raise RuntimeError('Missing NERO park preset; rebuild agx_arm_moveit first.')
        for joint in arm_park.findall('joint'):
            park.append(copy.deepcopy(joint))
    config = cfg.to_dict()
    config.update(robot_description=ET.tostring(robot, encoding='unicode'),
        robot_description_semantic=ET.tostring(semantic, encoding='unicode'),
        robot_description_kinematics=kinematics,
        moveit_simple_controller_manager=moveit_controllers)
    config['robot_description_planning']['joint_limits'] = limits
    controller_config['controller_manager'] = {'ros__parameters': controllers}
    return config, controller_config
