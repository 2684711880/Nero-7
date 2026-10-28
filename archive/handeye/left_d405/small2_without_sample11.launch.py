""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
' EYE-IN-HAND: gripper_flange -> handeye_color_optical '
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    nodes = [Node(package='tf2_ros', executable='static_transform_publisher', output='log', arguments=['--frame-id', 'gripper_flange', '--child-frame-id', 'handeye_color_optical', '--x', '0.06871607921931361', '--y', '-0.006907843463114035', '--z', '0.03605327980945122', '--qx', '-0.08674290648938406', '--qy', '-0.112828622345433', '--qz', '0.7026762222199935', '--qw', '0.6971309036902926'])]
    return LaunchDescription(nodes)
