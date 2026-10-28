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
                "0.0673125",
                "--y",
                "-0.00907703",
                "--z",
                "0.0541585",
                "--qx",
                "-0.11465",
                "--qy",
                "-0.125048",
                "--qz",
                "0.691352",
                "--qw",
                "0.702318",
                # "--roll",
                # "0.0125861",
                # "--pitch",
                # "-0.340729",
                # "--yaw",
                # "1.55723",
            ],
        ),
    ]
    return LaunchDescription(nodes)
