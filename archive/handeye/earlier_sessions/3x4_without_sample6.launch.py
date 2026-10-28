""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
' EYE-IN-HAND: gripper_flange -> handeye_color_optical '
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    nodes = [Node(package='tf2_ros', executable='static_transform_publisher', output='log', arguments=['--frame-id', 'gripper_flange', '--child-frame-id', 'handeye_color_optical', '--x', '0.07861236851397443', '--y', '-0.01391726315001962', '--z', '0.04321369005353384', '--qx', '-0.1068433425950823', '--qy', '-0.1263705109428412', '--qz', '0.6843111439067576', '--qw', '0.7101642432791024'])]
    return LaunchDescription(nodes)
