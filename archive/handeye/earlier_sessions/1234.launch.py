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
                "0.0691048",
                "--y",
                "-0.00691168",
                "--z",
                "0.0179042",
                "--qx",
                "-0.118162",
                "--qy",
                "-0.129421",
                "--qz",
                "0.687673",
                "--qw",
                "0.704553",
                # "--roll",
                # "0.0122475",
                # "--pitch",
                # "-0.352113",
                # "--yaw",
                # "1.54873",
            ],
        ),
    ]
    return LaunchDescription(nodes)
