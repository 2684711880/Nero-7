"""Open RViz on an already-running read-only fixed-bottle planning scene."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nero_dual_bringup.model import compose

def generate_launch_description():
    config,_=compose(.28)
    path=Path(get_package_share_directory('nero_dual_bringup'))/'config/dual.rviz'
    return LaunchDescription([Node(package='rviz2',executable='rviz2',output='screen',arguments=['-d',str(path)],parameters=[config])])
