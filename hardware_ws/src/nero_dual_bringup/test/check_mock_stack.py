"""Run only against mock_stack.launch.py in isolated ROS_DOMAIN_ID=93. No CAN."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory

rclpy.init()
node=Node('dual_wiring_integration')
received={'left':[],'right':[]}
feedback_names=set()
gates={'left':False,'right':False}
feedback={}
for side in received:
    node.create_subscription(JointState,f'/{side}/control/joint_states',
        lambda msg,s=side: received[s].append(msg),10)
    feedback[side]=node.create_publisher(JointState,f'/{side}/feedback/joint_states',10)
    def gate(req,res,s=side):
        gates[s]=req.data
        res.success=True
        return res
    node.create_service(SetBool,f'/{side}/control_enable',gate)
node.create_subscription(JointState,'/joint_states',lambda msg:feedback_names.update(msg.name),10)

def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        for side,pub in feedback.items():
            msg=JointState()
            msg.header.stamp=node.get_clock().now().to_msg()
            msg.name=[f'joint{i}' for i in range(1,8)]+['gripper']
            msg.position=[0.0]*7+[0.04]
            pub.publish(msg)
        rclpy.spin_once(node,timeout_sec=0.01)

def execute(side,part,joints,positions):
    client=ActionClient(node,FollowJointTrajectory,f'/{side}_{part}_controller/follow_joint_trajectory')
    assert client.wait_for_server(timeout_sec=10)
    goal=FollowJointTrajectory.Goal()
    goal.trajectory.joint_names=joints
    point=JointTrajectoryPoint()
    point.positions=positions
    point.time_from_start.sec=1
    goal.trajectory.points=[point]
    future=client.send_goal_async(goal)
    while not future.done(): spin(0.02)
    handle=future.result()
    assert handle.accepted
    result=handle.get_result_async()
    deadline=time.monotonic()+8
    while not result.done() and time.monotonic()<deadline: spin(0.02)
    assert result.done()
    assert result.result().result.error_code==0,result.result().result
    spin(0.3)

spin(1.0)
assert not received['left'] and not received['right'],'startup commands leaked'
assert all(f'{s}_joint{i}' in feedback_names for s in received for i in range(1,8))
execute('left','arm',[f'left_joint{i}' for i in range(1,8)],[0.02]*7)
assert len(received['left'])>0 and not received['right'],'left/right routing failed'
assert all(set(msg.name)=={f'joint{i}' for i in range(1,8)} for msg in received['left'])
left_count=len(received['left'])
execute('right','gripper',['right_gripper'],[0.06])
assert len(received['left'])==left_count
assert received['right'] and all(msg.name==['gripper'] for msg in received['right'])
report={'hardware_connected':False,'startup_commands':0,
    'combined_feedback_joints':sorted(feedback_names),
    'left_arm_command_frames':left_count,'right_gripper_command_frames':len(received['right']),
    'left_only_no_right_arm_commands':True,'gripper_only_no_arm_commands':True,
    'official_gate_closed_after_execution':not any(gates.values())}
assert report['official_gate_closed_after_execution']
planner=node.create_client(GetMotionPlan,'/plan_kinematic_path')
assert planner.wait_for_service(timeout_sec=5)
planned={}
for group in ('left_arm','right_arm','both_arms'):
    request=GetMotionPlan.Request()
    request.motion_plan_request.group_name=group
    request.motion_plan_request.allowed_planning_time=3.0
    request.motion_plan_request.num_planning_attempts=1
    request.motion_plan_request.start_state.joint_state.name=[s+'_'+f'joint{i}' for s in ('left','right') for i in range(1,8)]+['left_gripper','right_gripper']
    request.motion_plan_request.start_state.joint_state.position=[0.0]*14+[0.04,0.04]
    constraints=Constraints()
    for side in ('left','right') if group=='both_arms' else (group.split('_')[0],):
        for i in range(1,8):
            constraints.joint_constraints.append(JointConstraint(joint_name=f'{side}_joint{i}',
                position=0.02,tolerance_above=0.001,tolerance_below=0.001,weight=1.0))
    request.motion_plan_request.goal_constraints=[constraints]
    future=planner.call_async(request)
    deadline=time.monotonic()+10
    while not future.done() and time.monotonic()<deadline: spin(0.02)
    assert future.done(),group
    code=future.result().motion_plan_response.error_code.val
    planned[group]=code
    assert code==1,(group,code)
report['moveit_planning_error_codes']=planned
executor=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
assert executor.wait_for_server(timeout_sec=5)
before={s:len(received[s]) for s in received}
goal=ExecuteTrajectory.Goal(trajectory=future.result().motion_plan_response.trajectory)
sent=executor.send_goal_async(goal)
while not sent.done(): spin(0.02)
assert sent.result().accepted
finished=sent.result().get_result_async()
deadline=time.monotonic()+15
while not finished.done() and time.monotonic()<deadline: spin(0.02)
assert finished.done()
assert finished.result().result.error_code.val==1,finished.result().result
spin(0.3)
report['both_arms_execution_command_frames']={s:len(received[s])-before[s] for s in received}
assert all(count>0 for count in report['both_arms_execution_command_frames'].values())
assert not any(gates.values())
out=Path('/home/a/Nero-7/hardware_ws/validation/dual_moveit/mock_report.json')
out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
node.destroy_node()
rclpy.shutdown()
