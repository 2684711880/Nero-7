#!/usr/bin/env python3
"""Read-only startup and settled Gazebo body rocking, independent of LIO."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from gazebo_msgs.msg import ModelStates,LinkStates
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=25.);a=p.parse_args()
rclpy.init();n=rclpy.create_node('measure_actual_base_rocking',parameter_overrides=[Parameter('use_sim_time',value=True)])
rows=[];head=[]
def now():return n.get_clock().now().nanoseconds*1e-9
def state(m):
 if 'mobile_dual_nero' not in m.name:return
 i=m.name.index('mobile_dual_nero');pose=m.pose[i];v=m.twist[i];q=pose.orientation
 roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
 pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
 rows.append([now(),pose.position.x,pose.position.y,pose.position.z,roll,pitch,v.linear.x,v.linear.z,v.angular.y])
def links(m):
 names=[i for i,k in enumerate(m.name) if k=='mobile_dual_nero::mid360_link']
 if names:
  pose=m.pose[names[0]];head.append([now(),pose.position.x,pose.position.y,pose.position.z])
subs=[n.create_subscription(ModelStates,'/gazebo/model_states',state,100),n.create_subscription(LinkStates,'/gazebo/link_states',links,100)]
end=time.monotonic()+120
while (not rows or now()-rows[0][0]<a.seconds) and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
assert rows and now()-rows[0][0]>=a.seconds-.1,'No complete simulation capture'
x=np.array(rows);h=np.array(head);t=x[:,0]-x[0,0]
def summarize(mask):
 v=x[mask];return {'samples':len(v),'pitch_peak_to_peak_deg':float(np.rad2deg(np.ptp(v[:,5]))),
 'pitch_mean_deg':float(np.rad2deg(v[:,5].mean())),'roll_peak_to_peak_deg':float(np.rad2deg(np.ptp(v[:,4]))),
 'base_x_peak_to_peak_m':float(np.ptp(v[:,1])),'base_z_peak_to_peak_m':float(np.ptp(v[:,3])),
 'max_abs_pitch_velocity_rad_s':float(np.abs(v[:,8]).max())}
report={'seconds':float(t[-1]),'startup_0_3s':summarize(t<=3),'settled_15_25s':summarize(t>=15),
 'initial_base_z_m':float(x[0,3]),'scope':'Actual Gazebo physics without motion commands; startup includes controller activation'}
if len(h):report['settled_head_x_peak_to_peak_m']=float(np.ptp(h[h[:,0]>=x[0,0]+15,1]))
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(a.output.with_suffix('.npz'),base=x,head=h);print(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown()
