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
                               ('kd', '100'), ('minDepth', '0.001'), ('maxVel', '0.1')]:
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
