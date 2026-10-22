#!/usr/bin/env python3
"""Simulation-only bounded shoulder sweep, with MoveIt checks and raw ray statistics."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('probe_mid360_arm_occlusion',parameter_overrides=[Parameter('use_sim_time',value=True)])
latest={};data=[];phase='setup'
def stat(msg):
 row=json.loads(msg.data);row['phase']=phase;data.append(row)
subs=[n.create_subscription(JointState,'/joint_states',lambda m:latest.__setitem__('joints',m),10),
 n.create_subscription(ModelStates,'/gazebo/model_states',lambda m:latest.__setitem__('models',m),10),
 n.create_subscription(String,'/mid360/diagnostics',stat,10)]
clients={side:ActionClient(n,FollowJointTrajectory,f'/{side}_arm_controller/follow_joint_trajectory') for side in ('left','right')}
check=n.create_client(GetStateValidity,'/check_state_validity')
assert check.wait_for_service(timeout_sec=20)
wall_end=time.monotonic()+20
while not ('joints' in latest and 'models' in latest and data):
 assert time.monotonic()<wall_end,'Missing Gazebo/sensor feedback'
 rclpy.spin_once(n,timeout_sec=.1)
assert 'mobile_dual_nero' in latest['models'].name
assert abs(latest['models'].pose[latest['models'].name.index('mobile_dual_nero')].position.y)<.2
initial=dict(zip(latest['joints'].name,latest['joints'].position))
# Check the complete two-arm interpolated path before sending any trajectory.
for i in range(11):
 request=GetStateValidity.Request();request.robot_state.is_diff=True
 request.robot_state.joint_state=JointState()
 request.robot_state.joint_state.name=list(initial)
 request.robot_state.joint_state.position=[(1-i/10)*v+(i/10)*1.0 if k in ('left_joint2','right_joint2') else v for k,v in initial.items()]
 f=check.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
 assert f.done() and f.result().valid, f'Path collision at step {i}: {f.result()}'

def hold(seconds):
 end=n.get_clock().now().nanoseconds*1e-9+seconds;limit=time.monotonic()+30
 while n.get_clock().now().nanoseconds*1e-9<end:
  assert time.monotonic()<limit,'Simulation clock stopped'
  rclpy.spin_once(n,timeout_sec=.05)

def move(raised):
 goals=[]
 for side,client in clients.items():
  assert client.wait_for_server(timeout_sec=5)
  goal=FollowJointTrajectory.Goal()
  goal.trajectory.joint_names=[f'{side}_joint{i}' for i in range(1,8)]
  point=JointTrajectoryPoint()
  point.positions=[1.0 if raised and i==2 else initial[f'{side}_joint{i}'] for i in range(1,8)]
  point.time_from_start.sec=4
  goal.trajectory.points=[point]
  future=client.send_goal_async(goal);rclpy.spin_until_future_complete(n,future,timeout_sec=5)
  assert future.done() and future.result().accepted
  goals.append(future.result().get_result_async())
 for future in goals:
  rclpy.spin_until_future_complete(n,future,timeout_sec=15)
  assert future.done() and future.result().result.error_code==0, str(future.result())

phase='relaxed';hold(3)
phase='moving_out';move(True)
phase='raised';hold(3)
phase='moving_back';move(False)
phase='restored';hold(3)
summary={}
for label in ['relaxed','moving_out','raised','moving_back','restored']:
 rows=[r for r in data if r['phase']==label]
 summary[label]=dict(frames=len(rows),self_fraction=sum(r['self_hits'] for r in rows)/max(1,sum(r['rays'] for r in rows)),
  mean_returns=sum(r['returns'] for r in rows)/max(1,len(rows)))
report=dict(trajectory='both shoulders from current ~1.25 rad to 1.0 rad and back, 4 seconds each way',
 path_checks=11,summary=summary,completed=True)
(a.output/'arm_occlusion.json').write_text(json.dumps(report,indent=2)+'\n')
(a.output/'arm_ray_stats.json').write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
