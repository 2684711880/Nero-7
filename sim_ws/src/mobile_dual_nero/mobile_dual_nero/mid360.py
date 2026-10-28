"""MID360 fixture + Gazebo plugin. Mount and sensor/IMU extrinsics are distinct."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory


def moves_with_upper_body(preset):
    return preset in ('chest_down', 'head_tilt')


def mount_pose(preset, xyz='', rpy='', tilt_deg=''):
    mounts = yaml.safe_load((Path(get_package_share_directory('mobile_dual_nero')) /
                             'config/mid360_mounts.yaml').read_text())
    if preset not in mounts:
        raise ValueError(f'Unknown MID360 mount: {preset}')
    def vec(value, fallback):
        result = list(map(float, value.split())) if value.strip() else list(fallback)
        if len(result) != 3 or not all(math.isfinite(v) for v in result):
            raise ValueError('MID360 xyz/rpy must contain three finite numbers')
        return result
    fallback_rpy = list(mounts[preset]['rpy'])
    if preset == 'head_tilt' and str(tilt_deg).strip():
        tilt = float(tilt_deg)
        if not math.isfinite(tilt) or not -90 <= tilt <= 90:
            raise ValueError('mid360_tilt_deg must be in [-90, 90] degrees')
        fallback_rpy = [0., math.radians(tilt), 0.]
    # An explicit complete rpy override takes precedence over the tilt convenience argument.
    return vec(xyz, mounts[preset]['xyz']), vec(rpy, fallback_rpy)


def attach_mid360(root, preset, xyz='', rpy='', points_per_second=128000, upper_xyz=(0,0,.275), upper_rpy=(0,0,0), tilt_deg=''):
    xyz, rpy = mount_pose(preset, xyz, rpy, tilt_deg)
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
    parent = 'base_link'
    local_xyz, local_rpy = xyz, rpy
    if moves_with_upper_body(preset):
        import numpy as np
        def rotation(v):
            r,p,y = v; cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
            return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                             [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
        upper_R = rotation(upper_rpy)
        local_xyz = upper_R.T@(np.array(xyz)-np.array(upper_xyz))
        R = upper_R.T@rotation(rpy)
        p = math.asin(max(-1.,min(1.,-R[2,0])))
        if abs(math.cos(p)) > 1e-8:
            local_rpy = [math.atan2(R[2,1],R[2,2]),p,math.atan2(R[1,0],R[0,0])]
        else:
            local_rpy = [math.atan2(-R[1,2],R[1,1]),p,0.]
        parent = 'dual_nero_base_link'
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link='mid360_link')
    ET.SubElement(joint, 'origin', xyz=' '.join(map(str, local_xyz)), rpy=' '.join(map(str, local_rpy)))
    ET.SubElement(ET.SubElement(root, 'gazebo', reference='mid360_joint'), 'preserveFixedJoint').text = 'true'
    plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin',
                           name='nero_mid360', filename='libnero_mid360.so')
    for key, value in dict(link_name='mid360_link', frame_id='mid360_link',
                           points_per_second=points_per_second, scan_hz=10,
                           imu_hz=200, min_range=.10, max_range=40).items():
        ET.SubElement(plugin, key).text = str(value)
    return xyz, rpy
