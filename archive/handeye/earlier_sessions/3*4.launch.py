""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: gripper_flange -> handeye_color_optical """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "gripper_flange",
                "--child-frame-id",
                "handeye_color_optical",
                "--x",
                "0.0793231",
                "--y",
                "-0.0128994",
                "--z",
                "0.0409606",
                "--qx",
                "-0.105839",
                "--qy",
                "-0.126592",
                "--qz",
                "0.690866",
                "--qw",
                "0.703901",
                # "--roll",
                # "0.0274011",
                # "--pitch",
                # "-0.330438",
                # "--yaw",
                # "1.55667",
            ],
        ),
    ]
    return LaunchDescription(nodes)
