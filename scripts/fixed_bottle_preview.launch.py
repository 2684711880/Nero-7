"""Read-only live robot MoveIt preview; no hardware driver or trajectory controller."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from nero_dual_bringup.model import compose

def generate_launch_description():
    config,_=compose(.28)
    own=Path(get_package_share_directory('nero_dual_bringup'))
    nodes=[]
    for side,can in [('left','can1'),('right','can0')]:
        nodes.append(Node(package='nero_dual_bringup',executable='pointing_passive_arm',arguments=['--side',side,'--can',can],output='screen'))
    nodes.extend([
        Node(package='nero_dual_bringup',executable='joint_bridge',output='screen'),
        Node(package='nero_dual_bringup',executable='real_hole_preview_bridge',output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':config['robot_description']}]),
        Node(package='moveit_ros_move_group',executable='move_group',output='screen',parameters=[config,{'publish_robot_description_semantic':True,'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True}]),
        Node(package='rviz2',executable='rviz2',output='log',arguments=['-d',str(own/'config/dual.rviz')],parameters=[config]),
    ])
    root=Path(__file__).resolve().parents[1]
    python=str(root/'.venv-official/bin/python')
    startup=[ExecuteProcess(cmd=[python,str(root/'scripts/fixed_bottle_board.py'),'--inspection-allow-stale'],output='screen'),
             ExecuteProcess(cmd=[python,str(root/'scripts/fixed_bottle_tool_scene.py')],output='screen'),
             ExecuteProcess(cmd=[python,str(root/'scripts/fixed_bottle_cameras.py')],output='screen')]
    return LaunchDescription(nodes+[TimerAction(period=5.,actions=startup)])
