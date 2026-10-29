#!/usr/bin/env python3
"""Execute saved hole trajectories only on this Gazebo simulation."""
import argparse,json,time,os,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from rosidl_runtime_py.set_message import set_message_fields
from tf2_ros import Buffer,TransformListener
from scipy.spatial.transform import Rotation

class Executor(Node):
 def __init__(self):
  super().__init__('nero_hole_sim_executor');self.tf=Buffer();self.listener=TransformListener(self.tf,self);self.joints=None
  self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'joints',m),10)
  self.params=self.create_client(GetParameters,'/robot_state_publisher/get_parameters');self.action=ActionClient(self,ExecuteTrajectory,'/execute_trajectory')
 def wait(self,f,timeout):
  rclpy.spin_until_future_complete(self,f,timeout_sec=timeout)
  if not f.done() or f.result() is None:raise RuntimeError('Timeout or invalid ROS result')
  return f.result()
 def validate_sim(self):
  if not self.params.wait_for_service(timeout_sec=5):raise RuntimeError('Simulation model unavailable')
  v=self.wait(self.params.call_async(GetParameters.Request(names=['robot_description','use_sim_time'])),5).values
  robot=ET.fromstring(v[0].string_value);plugin=robot.findtext('ros2_control/hardware/plugin')
  if plugin!='gazebo_ros2_control/GazeboSystem' or not v[1].bool_value or robot.find("link[@name='sim_hole_plate']") is None or robot.find("link[@name='sim_right_rod_tip']") is None:raise RuntimeError('Not the expected simulated NERO hole scene')
 def transform(self,a,b):
  end=time.monotonic()+10
  while time.monotonic()<end:
   rclpy.spin_once(self,timeout_sec=.1)
   try:
    t=self.tf.lookup_transform(a,b,Time()).transform;q=t.rotation;T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return T
   except Exception:pass
  raise RuntimeError('TF unavailable '+a+' -> '+b)
 def run(self,plan):
  if os.environ.get('ROS_DOMAIN_ID')!='93' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('Use isolated simulation domain 93')
  self.validate_sim()
  if plan.get('status')!='planned' or plan.get('feed_fraction',0)<.999 or plan.get('source')!='sim_depth':raise ValueError('Need a complete simulation-only plan')
  end=time.monotonic()+5
  while self.joints is None and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.1)
  if self.joints is None:raise RuntimeError('No simulated joints')
  first=plan['approach']['joint_trajectory'];current=dict(zip(self.joints.name,self.joints.position));start=first['points'][0]['positions']
  initial=plan.get('initial_joint_positions',{})
  names=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
  if any(name not in current or name not in initial or abs(current[name]-initial[name])>.03 for name in names):raise RuntimeError('Simulation arms changed since plan; re-detect and replan')
  if max(abs(current[n]-v) for n,v in zip(first['joint_names'],start))>.03:raise RuntimeError('Simulation arm moved since plan; re-detect and replan')
  if not self.action.wait_for_server(timeout_sec=10):raise RuntimeError('MoveIt simulation execution unavailable')
  for key in ('approach','feed'):
   msg=RobotTrajectory();set_message_fields(msg,plan[key]);g=ExecuteTrajectory.Goal();g.trajectory=msg
   h=self.wait(self.action.send_goal_async(g),15)
   if not h.accepted:raise RuntimeError(key+' execution rejected')
   result=self.wait(h.get_result_async(),120).result
   if result.error_code.val!=1:raise RuntimeError(key+' execution failed: '+str(result.error_code.val))
   print(key,'executed in simulation',flush=True)
  # Compare simulated rod tip to the plate plane and centre after execution.
  T=self.transform('sim_hole_plate','sim_right_rod_tip');p=T[:3,3]
  return dict(status='executed_simulation',motion_sent_to_simulation=True,physical_hardware_commands=False,rod_tip_in_plate_m=p.tolist(),radial_error_mm=float(np.linalg.norm(p[:2])*1000),signed_plane_offset_mm=float(p[2]*1000))

def main():
 p=argparse.ArgumentParser();p.add_argument('--plan',type=Path,default=(Path(__file__).resolve().parents[1] / 'sim_ws/validation/hole_approach/latest/plan.json'));p.add_argument('--execute',action='store_true');a=p.parse_args()
 if not a.execute:raise SystemExit('Pass --execute to move the Gazebo model; no physical hardware commands.')
 rclpy.init();n=Executor()
 try:
  out=n.run(json.loads(a.plan.read_text()));(a.plan.parent/'execution.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2),flush=True)
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
