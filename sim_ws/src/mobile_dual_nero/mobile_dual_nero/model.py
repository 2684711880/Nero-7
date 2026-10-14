from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory
from ranger_air_gazebo.model import simulation_description

LINK_RENAMES = {'base_link': 'dual_nero_base_link', 'camera_link': 'dual_camera_link'}


def gazebo_model_paths():
    return [str(Path(get_package_share_directory(package)).parent)
            for package in ['mobile_dual_nero', 'ranger_air_description', 'dual_nero_moveit_config']]


def compose(mount_xyz='0 0 0.275', mount_rpy='0 0 0'):
    """Preserve both source descriptions; return one movable tree and one hardware system."""
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
    j = ET.SubElement(base, 'joint', name='dual_nero_mount_joint', type='fixed')
    ET.SubElement(j, 'parent', link='arm_mount_plate')
    ET.SubElement(j, 'child', link='dual_nero_base_link')
    ET.SubElement(j, 'origin', xyz='0 0 .006', rpy='0 0 0')
    base.find('ros2_control').set('name', 'MobileDualNeroSystem')
    base.find("gazebo/plugin[@name='gazebo_ros2_control']/parameters").text = str(share / 'config/controllers.yaml')
    semantic = ET.parse(dual / 'config/dual_nero.srdf').getroot()
    semantic.set('name', 'mobile_dual_nero')
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
    # Adjacent rigid mounts, and chassis/torso separated only by the thin adapter.
    for pair in [('base_link', 'arm_mount_plate'), ('arm_mount_plate', 'dual_nero_base_link')]:
        ET.SubElement(semantic, 'disable_collisions', link1=pair[0], link2=pair[1], reason='Adjacent')
    return ET.tostring(base, encoding='unicode'), ET.tostring(semantic, encoding='unicode')
