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
                "0.082761",
                "--y",
                "-0.0139977",
                "--z",
                "0.0387344",
                "--qx",
                "-0.11094",
                "--qy",
                "-0.128261",
                "--qz",
                "0.686439",
                "--qw",
                "0.707137",
                # "--roll",
                # "0.0203557",
                # "--pitch",
                # "-0.34023",
                # "--yaw",
                # "1.54459",
            ],
        ),
    ]
    return LaunchDescription(nodes)
