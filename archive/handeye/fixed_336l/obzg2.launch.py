""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: base_link -> orbbec336l_color_optical """
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
                "base_link",
                "--child-frame-id",
                "orbbec336l_color_optical",
                "--x",
                "0.0538397",
                "--y",
                "0.111754",
                "--z",
                "-0.0705497",
                "--qx",
                "0.578192",
                "--qy",
                "-0.412282",
                "--qz",
                "-0.564366",
                "--qw",
                "-0.420962",
                # "--roll",
                # "1.5618",
                # "--pitch",
                # "-2.83111",
                # "--yaw",
                # "-1.59499",
            ],
        ),
    ]
    return LaunchDescription(nodes)
