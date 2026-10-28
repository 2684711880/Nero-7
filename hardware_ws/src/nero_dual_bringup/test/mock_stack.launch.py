"""Offline integration fixture: excludes BOTH CAN drivers and RViz from the real launch."""
from pathlib import Path
import importlib.util
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    path=Path(get_package_share_directory('nero_dual_bringup'))/'launch/dual_moveit.launch.py'
    spec=importlib.util.spec_from_file_location('dual_launch',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    def mock_setup(context):
        return [action for action in module.setup(context)
                if not isinstance(action, IncludeLaunchDescription)
                and not (isinstance(action, Node) and action.node_package == 'rviz2')]
    real = module.generate_launch_description()
    return LaunchDescription([
        SetLaunchConfiguration('use_rviz', 'false'),
        *[action for action in real.entities if not isinstance(action, OpaqueFunction)],
        OpaqueFunction(function=mock_setup),
    ])
