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
                "0.079307",
                "--y",
                "-0.00557891",
                "--z",
                "0.0358009",
                "--qx",
                "-0.107807",
                "--qy",
                "-0.119767",
                "--qz",
                "0.702008",
                "--qw",
                "0.693699",
                # "--roll",
                # "0.0195996",
                # "--pitch",
                # "-0.32312",
                # "--yaw",
                # "1.5859",
            ],
        ),
    ]
    return LaunchDescription(nodes)
