#!/usr/bin/env python3
"""Plan or execute a fixed, simulation-only right-arm observation posture.

No plate link/pose, hole truth, or depth detection is used in this stage.
"""
import argparse,copy,json,os,time
from pathlib import Path
import rclpy
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import RobotState,Constraints,JointConstraint,DisplayTrajectory,RobotTrajectory
from moveit_msgs.action import ExecuteTrajectory
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from sim_hole_plan import Planner
from sim_hole_execute import Executor

VIEW=[.1076393,-.9554579,1.4951906,1.6442938,.2765393,-.1913224,.3182951]
OUT=(Path(__file__).resolve().parents[1] / 'sim_ws/validation/hole_approach/latest/observe_plan.json')

def check_domain():
 if os.environ.get('ROS_DOMAIN_ID')!='93' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('Use isolated simulation domain 93')

def plan():
 check_domain();n=Planner();guard=Executor()
 try:
  guard.validate_sim();end=time.monotonic()+10
  while n.joints is None and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
  if n.joints is None:raise RuntimeError('No simulated joint feedback')
  state=RobotState();state.joint_state=n.joints
  names=[f'right_joint{i}' for i in range(1,8)]; current=dict(zip(n.joints.name,n.joints.position))
  if not all(k in current for k in names):raise RuntimeError('Missing right-arm joints')
  req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='right_arm';m.pipeline_id='ompl';m.num_planning_attempts=5;m.allowed_planning_time=10.;m.max_velocity_scaling_factor=.1;m.max_acceleration_scaling_factor=.1;m.start_state=state
  c=Constraints()
  for name,value in zip(names,VIEW):c.joint_constraints.append(JointConstraint(joint_name=name,position=float(value),tolerance_above=.005,tolerance_below=.005,weight=1.))
  m.goal_constraints=[c];result=n.call(n.plan,req,45).motion_plan_response
  if result.error_code.val!=1:raise RuntimeError('Observation motion planning failed: '+str(result.error_code.val))
  display=DisplayTrajectory();display.model_id='dual_nero';display.trajectory_start=state;display.trajectory=[result.trajectory];n.pub.publish(display)
  report=dict(status='planned',source='predefined_observation_joints',uses_hole_pose=False,motion_sent=False,initial_joint_positions=current,trajectory=message_to_ordereddict(result.trajectory))
  OUT.parent.mkdir(parents=True,exist_ok=True);OUT.write_text(json.dumps(report,indent=2)+'\n');print('Observation pose planned; no hole truth used. Motion not sent.',flush=True);time.sleep(1)
 finally:guard.destroy_node();n.destroy_node()

def execute():
 check_domain();report=json.loads(OUT.read_text());n=Executor()
 try:
  n.validate_sim()
  if report.get('status')!='planned' or report.get('uses_hole_pose') is not False:raise ValueError('Need a valid observation plan')
  end=time.monotonic()+5
  while n.joints is None and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
  if n.joints is None:raise RuntimeError('No simulated joint feedback')
  now=dict(zip(n.joints.name,n.joints.position));before=report['initial_joint_positions'];names=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
  if any(k not in now or k not in before or abs(now[k]-before[k])>.03 for k in names):raise RuntimeError('Arms changed since observation plan')
  if not n.action.wait_for_server(timeout_sec=10):raise RuntimeError('Simulation execution unavailable')
  t=RobotTrajectory();set_message_fields(t,copy.deepcopy(report['trajectory']));g=ExecuteTrajectory.Goal();g.trajectory=t
  handle=n.wait(n.action.send_goal_async(g),10)
  if not handle.accepted:raise RuntimeError('Observation trajectory rejected')
  res=n.wait(handle.get_result_async(),120).result
  if res.error_code.val!=1:raise RuntimeError('Observation motion failed: '+str(res.error_code.val))
  report.update(status='executed_simulation',motion_sent_to_simulation=True,physical_hardware_commands=False)
  tmp=OUT.with_suffix('.json.tmp');tmp.write_text(json.dumps(report,indent=2)+'\n');tmp.replace(OUT)
  print('Observation pose reached in simulation. Now detect the hole.',flush=True)
 finally:n.destroy_node()

def main():
 p=argparse.ArgumentParser();p.add_argument('action',choices=['plan','execute']);a=p.parse_args();rclpy.init()
 try:plan() if a.action=='plan' else execute()
 finally:rclpy.shutdown()
if __name__=='__main__':main()
