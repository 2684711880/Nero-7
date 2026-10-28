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
                "0.0815296",
                "--y",
                "-0.0148923",
                "--z",
                "0.0377815",
                "--qx",
                "-0.113754",
                "--qy",
                "-0.128918",
                "--qz",
                "0.686333",
                "--qw",
                "0.706673",
                # "--roll",
                # "0.0172022",
                # "--pitch",
                # "-0.345165",
                # "--yaw",
                # "1.54459",
            ],
        ),
    ]
    return LaunchDescription(nodes)
