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
                "0.0823426",
                "--y",
                "-0.0159256",
                "--z",
                "0.0368999",
                "--qx",
                "-0.109393",
                "--qy",
                "-0.126176",
                "--qz",
                "0.67482",
                "--qw",
                "0.71884",
                # "--roll",
                # "0.0137879",
                # "--pitch",
                # "-0.335288",
                # "--yaw",
                # "1.50998",
            ],
        ),
    ]
    return LaunchDescription(nodes)
