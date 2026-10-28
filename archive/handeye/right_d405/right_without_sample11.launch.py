"""Park candidate excluding original sample 11; review before use."""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package="tf2_ros", executable="static_transform_publisher", output="log", arguments=['--frame-id', 'gripper_flange', '--child-frame-id', 'handeye_color_optical', '--x', '0.0826832789004441', '--y', '-0.008000462368711056', '--z', '0.032842341337766826', '--qx', '-0.12742381189733762', '--qy', '-0.12990090550149777', '--qz', '0.7104041972154379', '--qw', '0.6798638124581512'])])
