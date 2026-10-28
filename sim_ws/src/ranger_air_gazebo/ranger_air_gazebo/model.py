"""Build a simulation variant without modifying the upstream CAD URDF."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from .kinematics import WHEELS, RADIUS


def simulation_description():
    model = Path(get_package_share_directory('ranger_air_description'))
    share = Path(get_package_share_directory('ranger_air_gazebo'))
    root = ET.parse(model / 'urdf/ranger_air_description.urdf').getroot()
    # Keep CAD visuals/inertias; replace detailed contact meshes with primitives.
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            link.remove(collision)
        name = link.get('name')
        if name == 'base_link':
            collision = ET.SubElement(link, 'collision')
            ET.SubElement(collision, 'origin', xyz='0 0 0.18', rpy='0 0 0')
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'box', size='0.50 0.30 0.10')
        elif name in [w + '_wheel_link' for w in WHEELS]:
            collision = ET.SubElement(link, 'collision')
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'cylinder',
                          radius=str(RADIUS), length='0.051')
            gazebo = ET.SubElement(root, 'gazebo', reference=name)
            for tag, value in [('mu1', '1.0'), ('mu2', '1.0'), ('kp', '1000000'),
                               ('kd', '3000'), ('minDepth', '0.001'), ('maxVel', '0.1')]:
                ET.SubElement(gazebo, tag).text = value
    control = ET.SubElement(root, 'ros2_control', name='RangerAirSystem', type='system')
    ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = 'gazebo_ros2_control/GazeboSystem'
    for w in WHEELS:
        for kind, interface in [('steering', 'position'), ('wheel', 'velocity')]:
            name = f'{w}_{kind}_joint'
            j = ET.SubElement(control, 'joint', name=name)
            ET.SubElement(j, 'command_interface', name=interface)
            for state in ['position', 'velocity']:
                ET.SubElement(j, 'state_interface', name=state)
            source_joint = root.find(f"joint[@name='{name}']")
            ET.SubElement(source_joint, 'limit', effort='30', velocity='20')
            ET.SubElement(source_joint, 'dynamics', damping='0.05', friction='0.0')
    plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin',
                           name='gazebo_ros2_control', filename='libgazebo_ros2_control.so')
    ET.SubElement(plugin, 'parameters').text = str(share / 'config/controllers.yaml')
    return ET.tostring(root, encoding='unicode')


def contact_spawn_pose(description, clearance=.003):
    """Free-body initial pose aligned with the four zero-steer tire contact planes.

    Preserve upstream axle geometry, including its 1 mm rear height offset.
    Only choose the initial pose; never constrain the subsequently moving base.
    """
    import math
    import numpy as np
    if not math.isfinite(clearance) or not 0 <= clearance <= .05:
        raise ValueError('Spawn clearance must be in [0, 0.05] metres')
    root = ET.fromstring(description)
    parents = {j.find('child').get('link'): j for j in root.findall('joint')}

    def origin(element):
        T = np.eye(4)
        if element is None: return T
        r,p,y = map(float, element.get('rpy', '0 0 0').split())
        cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
        T[:3,:3] = [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                    [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
        T[:3,3] = list(map(float, element.get('xyz','0 0 0').split()))
        return T

    def pose(link):
        if link == 'base_link': return np.eye(4)
        joint = parents[link]
        return pose(joint.find('parent').get('link')) @ origin(joint.find('origin'))

    tires = []
    for wheel in WHEELS:
        name = wheel+'_wheel_link'
        collision = root.find(f"link[@name='{name}']/collision")
        cylinder = collision.find('geometry/cylinder')
        T = pose(name) @ origin(collision.find('origin'))
        tires.append((T[:3,3], T[:3,2], float(cylinder.get('radius')), float(cylinder.get('length'))/2))
    bottoms = np.array([[*center[:2], center[2]-radius*math.sqrt(max(0.,1-axis[2]**2))-half*abs(axis[2])]
                        for center,axis,radius,half in tires])
    a,b,_ = np.linalg.lstsq(np.column_stack((bottoms[:,:2],np.ones(4))),bottoms[:,2],rcond=None)[0]
    roll = -math.atan(b)
    pitch = math.asin(a/math.sqrt(1+a*a+b*b))
    dummy = ET.Element('origin', rpy=f'{roll} {pitch} 0')
    R = origin(dummy)[:3,:3]
    lowest = min((R@center)[2]-radius*math.sqrt(max(0.,1-(R@axis)[2]**2))-half*abs((R@axis)[2])
                 for center,axis,radius,half in tires)
    return float(clearance-lowest), float(roll), float(pitch)
