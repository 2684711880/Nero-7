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
                "0.0806875",
                "--y",
                "-0.00982326",
                "--z",
                "0.0383025",
                "--qx",
                "-0.12752",
                "--qy",
                "-0.131495",
                "--qz",
                "0.705684",
                "--qw",
                "0.684439",
                # "--roll",
                # "0.0118213",
                # "--pitch",
                # "-0.368245",
                # "--yaw",
                # "1.60356",
            ],
        ),
    ]
    return LaunchDescription(nodes)
