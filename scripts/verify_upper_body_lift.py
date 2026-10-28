#!/usr/bin/env python3
"""Simulation-only bounded lift acceptance using actual feedback and TF."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
from ranger_air_gazebo.lift_control import LiftControl

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mount',choices=['chest_down','chassis_up','head_tilt'],required=True)
p.add_argument('--tilt-deg',type=float,default=45.)
p.add_argument('--output',type=Path,required=True);a=p.parse_args()
rclpy.init();n=rclpy.create_node('verify_upper_body_lift',parameter_overrides=[Parameter('use_sim_time',value=True)])
lift=LiftControl(n);buf=Buffer();listener=TransformListener(buf,n)
latest={};counts={'odom':0,'filtered':0};estimated=[]
def odom(m):counts['odom']+=1
def filtered(m):counts['filtered']+=1

def mapping_tf(m):
 for t in m.transforms:
  if t.child_frame_id=='lio_robot/base_link':estimated.append([t.header.stamp.sec+t.header.stamp.nanosec/1e9,t.transform.translation.z])
subs=[n.create_subscription(Odometry,'/lio/odom',odom,10),
 n.create_subscription(String,'/mid360/self_filter/diagnostics',filtered,10),
 n.create_subscription(TFMessage,'/lio/tf',mapping_tf,100)]
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01);lift.tick()
end=time.monotonic()+40
while time.monotonic()<end:
 spin(.1)
 if lift.fresh() and lift.validity.service_is_ready() and counts['odom']>5:break
assert lift.fresh() and counts['odom']>5, 'Simulation/controllers/LIO not ready'
def snapshot():
 q=dict(zip(lift.feedback.name,lift.feedback.position))
 transforms={}
 for child in ['dual_nero_base_link','left_gripper_base','right_gripper_base','mid360_link']:
  t=buf.lookup_transform('base_link',child,rclpy.time.Time()).transform
  transforms[child]=[t.translation.x,t.translation.y,t.translation.z]
 return {'height':q[lift.JOINT],'transforms':transforms,'counts':counts.copy(),
         'estimated_base_z':estimated[-1][1] if estimated else None,
         'lidar_orientation_xyzw':[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]}
initial=snapshot();assert abs(initial['height'])<.02,'Expected lowered robot'
if a.mount=='head_tilt':
 expected=[0.,math.sin(math.radians(a.tilt_deg)/2),0.,math.cos(math.radians(a.tilt_deg)/2)]
 actual=initial['lidar_orientation_xyzw']
 assert min(sum((v-w)**2 for v,w in zip(actual,expected)),sum((v+w)**2 for v,w in zip(actual,expected)))<1e-8,actual
rows=[initial]
try:
 for delta in [.20,-.20]:
  lift.request(delta);assert lift.active,'Lift command was refused'
  end=time.monotonic()+70
  while lift.active and time.monotonic()<end:spin(.03)
  assert not lift.active,'Lift command timed out'
  spin(1.)
  rows.append(snapshot())
 assert abs(rows[1]['height']-initial['height']-.20)<.015
 assert abs(rows[2]['height']-initial['height'])<.015
 for link in ['dual_nero_base_link','left_gripper_base','right_gripper_base','mid360_link']:
  delta=rows[1]['transforms'][link][2]-rows[0]['transforms'][link][2]
  expected=0. if link=='mid360_link' and a.mount=='chassis_up' else .20
  assert abs(delta-expected)<.015,(link,delta,expected)
 assert rows[-1]['counts']['filtered']>rows[0]['counts']['filtered']+10
 assert rows[-1]['counts']['odom']>rows[0]['counts']['odom']+10
 report={'mount':a.mount,'passed':True,'sequence':'0 -> 0.20 -> 0 m', 'samples':rows,
         'base_motion_commanded':False,'scope':'Feedback, relative TF and live LIO/filter outputs; no full navigation accuracy claim'}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
finally:
 lift.stop();spin(.2);n.destroy_node();rclpy.shutdown()
