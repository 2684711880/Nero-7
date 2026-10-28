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
                "0.0437933",
                "--y",
                "0.112598",
                "--z",
                "-0.0789995",
                "--qx",
                "0.59681",
                "--qy",
                "-0.405238",
                "--qz",
                "-0.545272",
                "--qw",
                "-0.426941",
                # "--roll",
                # "1.52794",
                # "--pitch",
                # "-2.83184",
                # "--yaw",
                # "-1.65165",
            ],
        ),
    ]
    return LaunchDescription(nodes)
