""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
' EYE-IN-HAND: gripper_flange -> handeye_color_optical '
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    nodes = [Node(package='tf2_ros', executable='static_transform_publisher', output='log', arguments=['--frame-id', 'gripper_flange', '--child-frame-id', 'handeye_color_optical', '--x', '0.06986330433584791', '--y', '-0.007801301350600356', '--z', '0.03612372733734285', '--qx', '-0.09090162618235294', '--qy', '-0.1117504075569522', '--qz', '0.7038623407004351', '--qw', '0.6955764128490581'])]
    return LaunchDescription(nodes)
