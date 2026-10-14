#!/usr/bin/env python3
"""Negative regression: the captured below-ground goal must never be executable."""
import json
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity, GetMotionPlan
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
rclpy.init();n=rclpy.create_node('check_ground_regression');states=[]
s=n.create_subscription(JointState,'/joint_states',states.append,10)
for _ in range(100):
 rclpy.spin_once(n,timeout_sec=.1)
 if states:break
assert states,'No joint states'
goal=json.loads((Path(__file__).resolve().parents[1]/'evidence/jitter_repro.json').read_text())['left_joint_goal']
state=RobotState();state.joint_state=states[-1]
for i,v in enumerate(goal):state.joint_state.position[state.joint_state.name.index(f'left_joint{i+1}')]=v
c=n.create_client(GetStateValidity,'check_state_validity');assert c.wait_for_service(timeout_sec=30)
r=GetStateValidity.Request();r.robot_state=state
f=c.call_async(r);rclpy.spin_until_future_complete(n,f,timeout_sec=10);result=f.result();assert result and not result.valid
pairs=[(x.contact_body_1,x.contact_body_2) for x in result.contacts]
assert any('simulation_ground' in pair for pair in pairs),pairs
print('PASS: captured pose collides with planning ground:',pairs,flush=True)
c=n.create_client(GetMotionPlan,'plan_kinematic_path');assert c.wait_for_service(timeout_sec=10)
r=GetMotionPlan.Request();r.motion_plan_request.start_state.is_diff=True;r.motion_plan_request.group_name='left_arm';r.motion_plan_request.allowed_planning_time=2.0;r.motion_plan_request.num_planning_attempts=1
constraints=Constraints()
for i,v in enumerate(goal):
 j=JointConstraint();j.joint_name=f'left_joint{i+1}';j.position=v;j.tolerance_above=.0001;j.tolerance_below=.0001;j.weight=1.0;constraints.joint_constraints.append(j)
r.motion_plan_request.goal_constraints=[constraints]
f=c.call_async(r);rclpy.spin_until_future_complete(n,f,timeout_sec=20);result=f.result();assert result
assert result.motion_plan_response.error_code.val!=1
print('PASS: below-ground goal planning rejected; error code',result.motion_plan_response.error_code.val,flush=True)
n.destroy_node();rclpy.shutdown()
