#!/usr/bin/env python3
"""Plan NERO Gazebo-only two-stage hole approach from latest simulated depth."""
import argparse,copy,json,math,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer,TransformListener
from moveit_msgs.srv import GetMotionPlan,GetCartesianPath,GetStateValidity
from moveit_msgs.msg import Constraints,JointConstraint,DisplayTrajectory,RobotState
from geometry_msgs.msg import Pose
from rosidl_runtime_py.convert import message_to_ordereddict

class Planner(Node):
 def __init__(self):
  super().__init__('sim_hole_two_stage_planner');self.joints=None
  self.sub=self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'joints',m),10)
  self.tf=Buffer();self.listener=TransformListener(self.tf,self)
  self.plan=self.create_client(GetMotionPlan,'/plan_kinematic_path');self.cart=self.create_client(GetCartesianPath,'/compute_cartesian_path');self.valid=self.create_client(GetStateValidity,'/check_state_validity')
  self.pub=self.create_publisher(DisplayTrajectory,'/display_planned_path',10)
 def call(self,client,req,timeout=30):
  if not client.wait_for_service(timeout_sec=5):raise RuntimeError('MoveIt service unavailable')
  f=client.call_async(req);rclpy.spin_until_future_complete(self,f,timeout_sec=timeout)
  if not f.done() or f.result() is None:raise RuntimeError('MoveIt service timed out')
  return f.result()
 def lookup(self,parent,child):
  end=time.monotonic()+10
  while time.monotonic()<end:
   rclpy.spin_once(self,timeout_sec=.1)
   try:
    t=self.tf.lookup_transform(parent,child,Time()).transform;q=t.rotation
    T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return T
   except Exception:pass
  raise RuntimeError('Missing simulation TF '+parent+' -> '+child)
 def compute(self,measurement):
  if measurement.get('status')!='detected' or measurement.get('frame')!='sim_right_d405_optical':raise ValueError('Need successful simulated depth detection')
  if not 0 <= time.time()-measurement.get('observed_wall_time',0) < 20:raise ValueError('Detection is stale; capture a new frame')
  end=time.monotonic()+8
  while self.joints is None and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.1)
  if self.joints is None:raise RuntimeError('No simulated joint states')
  state=RobotState();state.joint_state=self.joints
  now=dict(zip(self.joints.name,self.joints.position));then=measurement.get('joint_positions',{})
  names=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
  if any(name not in now or name not in then or abs(now[name]-then[name])>.02 for name in names):raise ValueError('Arms moved after depth capture; capture again')
  check=GetStateValidity.Request();check.robot_state=state;check.group_name='';valid=self.call(self.valid,check)
  if not valid.valid:raise RuntimeError('Start state in collision: '+str([(c.contact_body_1,c.contact_body_2) for c in valid.contacts]))
  camera=self.lookup('base_link','sim_right_d405_optical');gripper=self.lookup('base_link','right_gripper_base')
  c=(camera@np.r_[measurement['center_m'],1])[:3];n=camera[:3,:3]@np.array(measurement['normal']);n/=np.linalg.norm(n)
  # Search redundant-arm IK while allowing tool roll, then collision-check both stages.
  urdf=Path(get_package_share_directory('dual_nero_moveit_config'))/'urdf/dual_nero.urdf'
  root=ET.parse(urdf).getroot();children={j.find('child').get('link'):j for j in root.findall('joint') if j.find('child') is not None}
  names=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
  observed=dict(zip(state.joint_state.name,state.joint_state.position))
  if not all(name in observed for name in names):raise RuntimeError('Missing dual-arm joint feedback')
  current=np.array([observed[name] for name in names[7:]])
  limits=[root.find(f"joint[@name='{name}']/limit") for name in names[7:]]
  lo=np.array([float(v.get('lower')) for v in limits]);hi=np.array([float(v.get('upper')) for v in limits])
  def fk(q):
   values=dict(observed);values.update(zip(names[7:],q));link='right_gripper_base';chain=[]
   while link in children:
    j=children[link];chain.append(j);link=j.find('parent').get('link')
   out=np.eye(4)
   for j in reversed(chain):
    o=j.find('origin');xyz=np.fromstring(o.get('xyz','0 0 0'),sep=' ') if o is not None else np.zeros(3);rpy=np.fromstring(o.get('rpy','0 0 0'),sep=' ') if o is not None else np.zeros(3)
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();t[:3,3]=xyz
    if j.get('type') in ('revolute','continuous'):
     axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');t[:3,:3]=t[:3,:3]@Rotation.from_rotvec(axis*values[j.get('name')]).as_matrix()
    out=out@t
   return out
  target=c+n*.20;desired_axis=-n
  def residual(q):
   g=fk(q);tip=(g@np.array([0,0,.18,1.]))[:3]
   return np.r_[(tip-target)*5,np.cross(g[:3,2],desired_axis)*2,.02*(q-current)]
  rng=np.random.default_rng(528);first=None;path=None;attempts=0
  for i in range(60):
   initial=current if i==0 else rng.uniform(lo*.85,hi*.85)
   result=least_squares(residual,np.clip(initial,lo+.001,hi-.001),bounds=(lo+.00001,hi-.00001),max_nfev=100,ftol=.001)
   g=fk(result.x);tip=(g@np.array([0,0,.18,1.]))[:3]
   if np.linalg.norm(tip-target)>.003 or np.linalg.norm(np.cross(g[:3,2],desired_axis))>.03 or np.dot(g[:3,2],desired_axis)<.99:continue
   trial=copy.deepcopy(state);mapping=dict(zip(trial.joint_state.name,trial.joint_state.position));mapping.update(zip(names[7:],result.x));trial.joint_state.position=[mapping[k] for k in trial.joint_state.name];trial.joint_state.velocity=[];trial.joint_state.effort=[]
   validity=GetStateValidity.Request();validity.robot_state=trial;validity.group_name=''
   if not self.call(self.valid,validity).valid:continue
   req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='right_arm';m.pipeline_id='ompl';m.num_planning_attempts=3;m.allowed_planning_time=4.;m.max_velocity_scaling_factor=.1;m.max_acceleration_scaling_factor=.1;m.start_state=state
   goal=Constraints()
   for name,value in zip(names[7:],result.x):goal.joint_constraints.append(JointConstraint(joint_name=name,position=float(value),tolerance_above=.005,tolerance_below=.005,weight=1.))
   m.goal_constraints=[goal];first=self.call(self.plan,req,20).motion_plan_response;attempts+=1
   if first.error_code.val!=1:continue
   traj=first.trajectory.joint_trajectory;mapping.update(zip(traj.joint_names,traj.points[-1].positions));trial.joint_state.position=[mapping[k] for k in trial.joint_state.name]
   end=c-n*.03;pos=end-g[:3,:3]@np.array([0,0,.18]);quat=Rotation.from_matrix(g[:3,:3]).as_quat();endpose=Pose();endpose.position.x,endpose.position.y,endpose.position.z=map(float,pos);endpose.orientation.x,endpose.orientation.y,endpose.orientation.z,endpose.orientation.w=map(float,quat)
   cr=GetCartesianPath.Request();cr.header.frame_id='base_link';cr.start_state=trial;cr.group_name='right_arm';cr.link_name='right_gripper_base';cr.waypoints=[endpose];cr.max_step=.003;cr.jump_threshold=0.;cr.avoid_collisions=True
   path=self.call(self.cart,cr,45)
   if path.fraction>.999:break
  if first is None or first.error_code.val!=1 or path is None or path.fraction<.999:raise RuntimeError('No complete collision-checked approach and feed path')
  report=dict(status='planned' if path.fraction>.999 else 'feed_incomplete',source='sim_depth',motion_sent=False,hole_center_base_m=c.tolist(),outward_normal_base=n.tolist(),approach_clearance_m=.20,feed_distance_m=.23,final_plane_offset_m=-.03,approach_points=len(traj.points),feed_fraction=path.fraction,feed_points=len(path.solution.joint_trajectory.points),start_valid=True,planning_attempts=attempts,initial_joint_positions=observed)
  display=DisplayTrajectory();display.model_id='dual_nero';display.trajectory_start=state;display.trajectory=[first.trajectory,path.solution];self.pub.publish(display)
  return report,first.trajectory,path.solution

def main():
 p=argparse.ArgumentParser();p.add_argument('--detection',type=Path,default=(Path(__file__).resolve().parents[1] / 'sim_ws/validation/hole_approach/latest/detection.json'));p.add_argument('--report',type=Path,default=(Path(__file__).resolve().parents[1] / 'sim_ws/validation/hole_approach/latest/plan.json'));a=p.parse_args()
 rclpy.init();n=Planner()
 try:
  report,first,feed=n.compute(json.loads(a.detection.read_text()));report.update(approach=message_to_ordereddict(first),feed=message_to_ordereddict(feed));a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('approach','feed')},indent=2),flush=True);time.sleep(1)
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
