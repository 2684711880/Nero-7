"""Single-arm calibrated model + official trajectory controller/driver + RViz menus."""
import json
from pathlib import Path
import sys
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
sys.path.insert(0,str(Path(__file__).parent))
from pointing_model import ROOT, compose


def setup(context):
    val=lambda k:LaunchConfiguration(k).perform(context)
    offset=json.loads(val('tcp_offset_mm'))
    if len(offset)!=3:raise ValueError('TCP offset needs 3 values')
    initial=json.loads(val('initial_q'))
    mock=val('mock')=='true'
    config,controllers=compose([v/1000. for v in offset],initial)
    calibration=json.loads((ROOT/'config/handeye/can1_d405.json').read_text())
    if not mock and calibration['can']!=val('can'):
        raise ValueError('Hand-eye result belongs to a different CAN interface')
    session=Path(val('session'));session.mkdir(parents=True,exist_ok=True)
    controller_file=session/'controllers.yaml';controller_file.write_text(yaml.safe_dump(controllers))
    params=session/'model.yaml';params.write_text(yaml.safe_dump({'/**':{'ros__parameters':config}}))
    common=dict(output='screen')
    nodes=[Node(package='moveit_ros_move_group',executable='move_group',parameters=[config],
                remappings=[('joint_states','/joint_states')],**common),
           Node(package='robot_state_publisher',executable='robot_state_publisher',
                parameters=[{'robot_description':config['robot_description']}],
                remappings=[('joint_states','/joint_states')],**common),
           Node(package='controller_manager',executable='ros2_control_node',
                parameters=[{'robot_description':config['robot_description']},str(controller_file)],
                remappings=[('joint_states','/dual_control/joint_states')],**common)]
    for name in ('joint_state_broadcaster','left_arm_controller'):
        nodes.append(Node(package='controller_manager',executable='spawner',arguments=[name],**common))
    if not mock:
        nodes += [Node(package='agx_arm_ctrl',executable='agx_arm_ctrl_single',namespace='left',
                     parameters=[dict(can_port=val('can'),arm_type='nero',effector_type='none',fw_version='v121',
                         auto_enable=False,control_enabled=False,fast_mode=False,speed_percent=0,pub_rate=200)],**common),
                  Node(package='agx_arm_moveit',executable='agx_arm_control_gate',namespace='left',
                       parameters=[{'status_topics':['/left_arm_controller/follow_joint_trajectory/_action/status'],
                                    'gate_service_name':'control_enable'}],**common),
                  Node(package='nero_dual_bringup',executable='joint_bridge',**common),
                  Node(package='nero_dual_bringup',executable='pointing_passive_arm',
                       arguments=['--can',val('other_can')],**common)]
        nodes.append(Node(package='nero_dual_bringup',executable='pointing_camera',
                          arguments=['--can',val('can'),'--serial',calibration['camera_serial'],
                                     '--session',str(session),'--no-robot-tf'],**common))
    nodes.append(Node(package='nero_dual_bringup',executable='pointing_server',arguments=[
        '--can',val('can'),'--tcp-offset-mm',*map(str,offset),'--standoff',val('standoff'),
        '--session',str(session),*(['--mock','--initial-q',*map(str,initial)] if mock else [])],**common))
    if val('rviz')=='true':
        rviz=Node(package='rviz2',executable='rviz2',arguments=['-d',str(ROOT/'config/pointing.rviz')],
                  parameters=[config],output='log')
        nodes.append(rviz)
        nodes.append(RegisterEventHandler(OnProcessExit(target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='Pointing RViz closed'))])))
    # A failed camera/server must not leave a half-started controller stack around.
    for action in list(nodes):
        if isinstance(action,Node) and action.node_package=='nero_dual_bringup':
            nodes.append(RegisterEventHandler(OnProcessExit(target_action=action,
                on_exit=[EmitEvent(event=Shutdown(reason='Pointing input/server exited'))])))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('can'),DeclareLaunchArgument('tcp_offset_mm'),
        DeclareLaunchArgument('session'),DeclareLaunchArgument('other_can',default_value='can0'),
        DeclareLaunchArgument('standoff',default_value='0.10'),
        DeclareLaunchArgument('mock',default_value='false'),DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('initial_q',default_value='[0,1.2,0,0.3,0,0,0]'),OpaqueFunction(function=setup)])
