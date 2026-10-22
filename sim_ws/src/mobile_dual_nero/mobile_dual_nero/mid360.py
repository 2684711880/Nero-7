"""MID360 fixture + Gazebo plugin. Mount and sensor/IMU extrinsics are distinct."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory


def mount_pose(preset, xyz='', rpy=''):
    mounts = yaml.safe_load((Path(get_package_share_directory('mobile_dual_nero')) /
                             'config/mid360_mounts.yaml').read_text())
    if preset not in mounts:
        raise ValueError(f'Unknown MID360 mount: {preset}')
    def vec(value, fallback):
        result = list(map(float, value.split())) if value.strip() else list(fallback)
        if len(result) != 3 or not all(math.isfinite(v) for v in result):
            raise ValueError('MID360 xyz/rpy must contain three finite numbers')
        return result
    return vec(xyz, mounts[preset]['xyz']), vec(rpy, mounts[preset]['rpy'])


def attach_mid360(root, preset, xyz='', rpy='', points_per_second=128000):
    xyz, rpy = mount_pose(preset, xyz, rpy)
    points_per_second = int(points_per_second)
    if not 1000 <= points_per_second <= 200000:
        raise ValueError('mid360_points_per_second must be in [1000, 200000]')
    link = ET.SubElement(root, 'link', name='mid360_link')
    inertia = ET.SubElement(link, 'inertial')
    ET.SubElement(inertia, 'mass', value='.265')
    ET.SubElement(inertia, 'inertia', ixx='.000173', ixy='0', ixz='0',
                  iyy='.000173', iyz='0', izz='.000187')
    for tag in ('visual', 'collision'):
        el = ET.SubElement(link, tag)
        ET.SubElement(ET.SubElement(el, 'geometry'), 'box', size='.065 .065 .060')
        if tag == 'visual':
            ET.SubElement(ET.SubElement(el, 'material', name='mid360_white'), 'color', rgba='.9 .93 .92 1')
    cap = ET.SubElement(link, 'visual')
    ET.SubElement(cap, 'origin', xyz='0 0 .034')
    ET.SubElement(ET.SubElement(cap, 'geometry'), 'cylinder', radius='.029', length='.008')
    ET.SubElement(ET.SubElement(cap, 'material', name='mid360_lens'), 'color', rgba='.06 .19 .24 1')
    joint = ET.SubElement(root, 'joint', name='mid360_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='mid360_link')
    ET.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
    ET.SubElement(ET.SubElement(root, 'gazebo', reference='mid360_joint'), 'preserveFixedJoint').text = 'true'
    plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin',
                           name='nero_mid360', filename='libnero_mid360.so')
    for key, value in dict(link_name='mid360_link', frame_id='mid360_link',
                           points_per_second=points_per_second, scan_hz=10,
                           imu_hz=200, min_range=.10, max_range=40).items():
        ET.SubElement(plugin, key).text = str(value)
    return xyz, rpy
