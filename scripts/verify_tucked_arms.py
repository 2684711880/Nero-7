#!/usr/bin/env python3
"""Simulation-only: check tucked arm envelope, collision samples and one small swing."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from moveit_msgs.srv import GetStateValidity
from mobile_dual_nero.model import compose
from nero_mid360_sim.body_geometry import BodyModel, collision_points
from ranger_air_gazebo.arm_swing import ArmSwing

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
rclpy.init();n=rclpy.create_node('verify_tucked_arms');s=ArmSwing(n)
end=time.monotonic()+30
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.01)
 if s.fresh() and s.validity.service_is_ready():break
assert s.fresh() and s.validity.service_is_ready()
q=dict(zip(s.feedback.name,s.feedback.position))
assert all(abs(q[f'{side}_joint2']-1.45)<.02 for side in ['left','right']),q
checks=0
states=[]
for j2 in np.linspace(1.25,1.45,11):states.append({**q,'left_joint2':float(j2),'right_joint2':float(j2)})
for height in [0.,.8]:
 for j1 in np.linspace(-.3,.3,17):
  states.append({**q,'left_joint1':float(j1),'right_joint1':float(j1),'upper_body_lift_joint':height})
for state in states:
 request=GetStateValidity.Request();request.robot_state.is_diff=True
 request.robot_state.joint_state.name=list(state);request.robot_state.joint_state.position=list(state.values())
 future=s.validity.call_async(request);rclpy.spin_until_future_complete(n,future,timeout_sec=3)
 assert future.done() and future.result().valid, str(future.result())
 checks+=1
root=ET.fromstring(compose(mid360_mount='head_tilt')[0]);model=BodyModel(ET.tostring(root,encoding='unicode'),0)
poses=model.poses(q);extents=[]
for link in root.findall('link'):
 if link.get('name').startswith(('left_','right_')):
  for c in link.findall('collision'):
   points=collision_points(c);T=poses[link.get('name')];ys=points@T[1,:3]+T[1,3]
   extents.append((float(ys.min()),float(ys.max())))
width=max(v[1] for v in extents)-min(v[0] for v in extents)
s.toggle();assert s.enabled,'Tucked pose failed keyboard swing gate'
end=time.monotonic()+60;cycle_end=None;observed=[]
try:
 while time.monotonic()<end and s.enabled:
  rclpy.spin_once(n,timeout_sec=.01);s.tick()
  if s.finish_ns and cycle_end is None:cycle_end=s.finish_ns
  actual=dict(zip(s.feedback.name,s.feedback.position))
  observed.append([actual['left_joint1'],actual['right_joint1']])
  if cycle_end and s.clock_ns>=cycle_end:break
 assert cycle_end and s.clock_ns>=cycle_end,'Swing did not complete'
finally:
 s.stop()
report={'passed':True,'j2_rad':{side:q[f'{side}_joint2'] for side in ['left','right']},
 'arm_envelope_width_m':width,'nominal_clearance_each_side_m':(1.2-width)/2,
 'collision_sample_checks':checks,'swing_completed':True,
 'observed_joint1_ranges':[[min(v[i] for v in observed),max(v[i] for v in observed)] for i in [0,1]],
 'scope':'Initial pose envelope and sampled collision states at base start position, one swing; not aisle driving acceptance'}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown()
