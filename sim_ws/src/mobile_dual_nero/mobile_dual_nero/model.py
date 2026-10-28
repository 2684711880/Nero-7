from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory
from ranger_air_gazebo.model import simulation_description

# A natural hanging pose; positive joint2 lowers both mirrored shoulders.
# Keep home/ready semantics intact for existing task files.
LIFT_MAX = 0.80

RELAXED_ARM = (0.0, 1.45, 0.0, 0.0, 0.0, 0.0, 0.0)

LINK_RENAMES = {'base_link': 'dual_nero_base_link', 'camera_link': 'dual_camera_link'}


def gazebo_model_paths():
    return [str(Path(get_package_share_directory(package)).parent)
            for package in ['mobile_dual_nero', 'ranger_air_description', 'dual_nero_moveit_config']]


def compose(mount_xyz='0 0 0.275', mount_rpy='0 0 0', startup_arm_pose='relaxed',
            mid360_mount='none', mid360_xyz='', mid360_rpy='', mid360_points_per_second=128000, lift_height=0.0, mid360_tilt_deg=''):
    """Preserve both source descriptions; return one movable tree and one hardware system."""
    if startup_arm_pose not in ('relaxed', 'home'):
        raise ValueError('startup_arm_pose must be relaxed or home')
    import math
    lift_height = float(lift_height)
    if not math.isfinite(lift_height) or not 0 <= lift_height <= LIFT_MAX:
        raise ValueError('lift_height must be in [0, 0.8] metres')
    dual = Path(get_package_share_directory('dual_nero_moveit_config'))
    share = Path(get_package_share_directory('mobile_dual_nero'))
    base = ET.fromstring(simulation_description())
    base.set('name', 'mobile_dual_nero')
    arm = ET.fromstring(xacro.process_file(str(dual / 'urdf/dual_nero.urdf.xacro'),
                                         mappings={'use_gazebo': 'true'}).toxml())
    used_joints = {j.get('name') for j in base.findall('joint')}
    joint_renames = {j.get('name'): 'dual_' + j.get('name') for j in arm.findall('joint')
                     if j.get('name') in used_joints}
    for element in arm:
        if element.tag == 'link':
            if element.get('name') == 'world':
                continue
            element.set('name', LINK_RENAMES.get(element.get('name'), element.get('name')))
            base.append(deepcopy(element))
        elif element.tag == 'joint':
            if element.find('parent').get('link') == 'world':
                continue
            element.set('name', joint_renames.get(element.get('name'), element.get('name')))
            for tag in ['parent', 'child']:
                item = element.find(tag)
                item.set('link', LINK_RENAMES.get(item.get('link'), item.get('link')))
            base.append(deepcopy(element))
        elif element.tag == 'ros2_control':
            for joint in element.findall('joint'):
                base.find('ros2_control').append(deepcopy(joint))
    for side in ('left', 'right'):
        for i, angle in enumerate(RELAXED_ARM, 1):
            initial = base.find(f"ros2_control/joint[@name='{side}_joint{i}']/state_interface[@name='position']/param[@name='initial_value']")
            initial.text = str(angle if startup_arm_pose == 'relaxed' else 0.0)
    # Thin adapter plate on the chassis; plate top coincides with the torso base.
    xyz = [float(x) for x in mount_xyz.split()]
    if len(xyz) != 3 or len(mount_rpy.split()) != 3:
        raise ValueError('Mount xyz/rpy must each contain three numbers')
    plate = ET.SubElement(base, 'link', name='arm_mount_plate')
    inertial = ET.SubElement(plate, 'inertial')
    ET.SubElement(inertial, 'mass', value='0.6')
    ET.SubElement(inertial, 'inertia', ixx='.0031322', ixy='0', ixz='0',
                  iyy='.0039272', iyz='0', izz='.007045')
    for tag in ['visual', 'collision']:
        item = ET.SubElement(plate, tag)
        ET.SubElement(ET.SubElement(item, 'geometry'), 'box', size='0.28 0.25 0.012')
        if tag == 'visual':
            ET.SubElement(ET.SubElement(item, 'material', name='mount_grey'), 'color', rgba='.25 .25 .28 1')
    j = ET.SubElement(base, 'joint', name='air_mount_joint', type='fixed')
    ET.SubElement(j, 'parent', link='base_link')
    ET.SubElement(j, 'child', link='arm_mount_plate')
    # Offset along the plate's local z axis, including optional mount rotation.
    import math
    roll, pitch, yaw = [float(x) for x in mount_rpy.split()]
    zaxis = (math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll),
             math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll),
             math.cos(pitch)*math.cos(roll))
    center = [v-.006*a for v, a in zip(xyz, zaxis)]
    ET.SubElement(j, 'origin', xyz=' '.join(str(v) for v in center), rpy=mount_rpy)
    j = ET.SubElement(base, 'joint', name='upper_body_lift_joint', type='prismatic')
    ET.SubElement(j, 'parent', link='arm_mount_plate')
    ET.SubElement(j, 'child', link='dual_nero_base_link')
    ET.SubElement(j, 'origin', xyz='0 0 .006', rpy='0 0 0')
    ET.SubElement(j, 'axis', xyz='0 0 1')
    ET.SubElement(j, 'limit', lower='0', upper=str(LIFT_MAX), effort='2500', velocity='0.05')
    ET.SubElement(j, 'dynamics', damping='150', friction='10')
    controlled = ET.SubElement(base.find('ros2_control'), 'joint', name='upper_body_lift_joint')
    ET.SubElement(controlled, 'command_interface', name='position')
    state = ET.SubElement(controlled, 'state_interface', name='position')
    ET.SubElement(state, 'param', name='initial_value').text = str(lift_height)
    ET.SubElement(controlled, 'state_interface', name='velocity')
    # Ideal vertical carriage plus a fixed rear guide; actuator internals are not modelled.
    rail = ET.SubElement(base, 'link', name='upper_body_lift_rail')
    inertial = ET.SubElement(rail, 'inertial')
    ET.SubElement(inertial, 'origin', xyz='-.30 0 .43', rpy='0 0 0')
    ET.SubElement(inertial, 'mass', value='4')
    # Uniform .06 x .14 x .86 m guide, inertia expressed at its own COM.
    ET.SubElement(inertial, 'inertia', ixx=str(4*(.14**2+.86**2)/12),
                  ixy='0', ixz='0', iyy=str(4*(.06**2+.86**2)/12),
                  iyz='0', izz=str(4*(.06**2+.14**2)/12))
    for tag in ('visual', 'collision'):
        part = ET.SubElement(rail, tag)
        ET.SubElement(part, 'origin', xyz='-.30 0 .43')
        ET.SubElement(ET.SubElement(part, 'geometry'), 'box', size='.06 .14 .86')
        if tag == 'visual':
            ET.SubElement(ET.SubElement(part, 'material', name='lift_steel'), 'color', rgba='.22 .25 .28 1')
    fixed = ET.SubElement(base, 'joint', name='upper_body_lift_rail_mount', type='fixed')
    ET.SubElement(fixed, 'parent', link='arm_mount_plate')
    ET.SubElement(fixed, 'child', link='upper_body_lift_rail')
    ET.SubElement(fixed, 'origin', xyz='0 0 .006')
    base.find('ros2_control').set('name', 'MobileDualNeroSystem')
    base.find("gazebo/plugin[@name='gazebo_ros2_control']/parameters").text = str(share / 'config/controllers.yaml')
    # Keep the hand link available to the Gazebo grasp constraint.
    hand_gazebo = ET.SubElement(base, 'gazebo', reference='right_gripper_base_joint')
    ET.SubElement(hand_gazebo, 'preserveFixedJoint').text = 'true'
    if mid360_mount != 'none':
        from .mid360 import attach_mid360
        attach_mid360(base, mid360_mount, mid360_xyz, mid360_rpy, mid360_points_per_second,
                      upper_xyz=xyz, upper_rpy=list(map(float, mount_rpy.split())), tilt_deg=mid360_tilt_deg)
    semantic = ET.parse(dual / 'config/dual_nero.srdf').getroot()
    semantic.set('name', 'mobile_dual_nero')
    for group, sides in [('left_arm', ('left',)), ('right_arm', ('right',)),
                         ('both_arms', ('left', 'right'))]:
        relaxed = ET.SubElement(semantic, 'group_state', name='relaxed', group=group)
        for side in sides:
            for i, angle in enumerate(RELAXED_ARM, 1):
                ET.SubElement(relaxed, 'joint', name=f'{side}_joint{i}', value=str(angle))
    for item in list(semantic.findall('disable_collisions')):
        if 'world' in (item.get('link1'), item.get('link2')):
            semantic.remove(item)
            continue
        for attr in ['link1', 'link2']:
            item.set(attr, LINK_RENAMES.get(item.get(attr), item.get(attr)))
    ET.SubElement(semantic, 'virtual_joint', name='mobile_base_joint', type='planar',
                  parent_frame='odom', child_link='base_link')
    for joint in base.findall('joint'):
        if joint.get('name') in used_joints and joint.get('type') != 'fixed':
            ET.SubElement(semantic, 'passive_joint', name=joint.get('name'))
    group = ET.SubElement(semantic, 'group', name='upper_body_lift')
    ET.SubElement(group, 'joint', name='upper_body_lift_joint')
    for name, height in [('lowered', 0.0), ('raised', LIFT_MAX)]:
        state = ET.SubElement(semantic, 'group_state', name=name, group='upper_body_lift')
        ET.SubElement(state, 'joint', name='upper_body_lift_joint', value=str(height))
    # Adjacent rigid mounts, and chassis/torso separated only by the thin adapter.
    for pair in [('base_link', 'arm_mount_plate'), ('arm_mount_plate', 'dual_nero_base_link'),
                 ('base_link', 'upper_body_lift_rail'), ('arm_mount_plate', 'upper_body_lift_rail')]:
        ET.SubElement(semantic, 'disable_collisions', link1=pair[0], link2=pair[1], reason='Adjacent')
    return ET.tostring(base, encoding='unicode'), ET.tostring(semantic, encoding='unicode')
