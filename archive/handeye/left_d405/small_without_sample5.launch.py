""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
' EYE-IN-HAND: gripper_flange -> handeye_color_optical '
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    nodes = [Node(package='tf2_ros', executable='static_transform_publisher', output='log', arguments=['--frame-id', 'gripper_flange', '--child-frame-id', 'handeye_color_optical', '--x', '0.08148901839057966', '--y', '-0.01498772668702807', '--z', '0.0368315076654722', '--qx', '-0.1136392211184907', '--qy', '-0.1292415094784447', '--qz', '0.6877768360004366', '--qw', '0.7052274693405981'])]
    return LaunchDescription(nodes)
