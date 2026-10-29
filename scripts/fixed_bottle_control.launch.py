"""Official control launch plus previously observed preview objects; no trajectory is sent."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

ROOT=Path(__file__).resolve().parents[1]
def generate_launch_description():
    original=Path(get_package_share_directory('nero_dual_bringup'))/'launch/dual_moveit.launch.py'
    control=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(original)),launch_arguments={'left_passive':'true','auto_enable':'false','cabinet_height':'0.0','support_height':'0.28'}.items())
    python=str(ROOT/'.venv-official/bin/python')
    board=ExecuteProcess(cmd=[python,str(ROOT/'scripts/fixed_bottle_board.py'),'--inspection-allow-stale'],output='screen')
    bottle=ExecuteProcess(cmd=[python,str(ROOT/'scripts/fixed_bottle_tool_scene.py')],output='screen')
    cameras=ExecuteProcess(cmd=[python,str(ROOT/'scripts/fixed_bottle_cameras.py')],output='screen')
    return LaunchDescription([control,TimerAction(period=5.0,actions=[board,bottle,cameras])])
