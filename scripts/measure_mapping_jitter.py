#!/usr/bin/env python3
"""Read-only stationary simulation jitter capture; truth is evaluation only."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from sensor_msgs.msg import Imu,JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf2_msgs.msg import TFMessage
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=20.);a=p.parse_args()
rclpy.init();n=rclpy.create_node('measure_stationary_mapping_jitter');rows={k:[] for k in ['imu','lio','robot','truth','lift']};now=[None];record=[False]
def stamp(s):return s.sec+s.nanosec*1e-9
def imu(m):
 now[0]=stamp(m.header.stamp)
 if record[0]:
  v=m.linear_acceleration;w=m.angular_velocity;rows['imu'].append([now[0],v.x,v.y,v.z,w.x,w.y,w.z])
def odom(m):
 if record[0]:
  q=m.pose.pose.orientation;v=m.pose.pose.position;rows['lio'].append([stamp(m.header.stamp),v.x,v.y,v.z,q.x,q.y,q.z,q.w])
def tf(m):
 if record[0]:
  for t in m.transforms:
   if t.child_frame_id=='lio_robot/base_link':rows['robot'].append([stamp(t.header.stamp),t.transform.translation.z])
def models(m):
 if record[0] and 'mobile_dual_nero' in m.name:
  v=m.pose[m.name.index('mobile_dual_nero')].position;rows['truth'].append([now[0] or 0,v.x,v.y,v.z])
def joints(m):
 if record[0] and 'upper_body_lift_joint' in m.name:rows['lift'].append([stamp(m.header.stamp),m.position[m.name.index('upper_body_lift_joint')]])
subs=[n.create_subscription(Imu,'/mid360/imu',imu,500),n.create_subscription(Odometry,'/lio/odom',odom,100),n.create_subscription(TFMessage,'/lio/tf',tf,100),n.create_subscription(ModelStates,'/gazebo/model_states',models,100),n.create_subscription(JointState,'/joint_states',joints,100)]
# Allow sensor + LIO initialization and an additional 3 simulation seconds to settle.
end=time.monotonic()+45
while now[0] is None and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
assert now[0] is not None,'No IMU'
ready=now[0]+3
while now[0]<ready and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
record[0]=True;start=now[0];end=time.monotonic()+max(60,a.seconds*6)
while now[0]-start<a.seconds and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
assert now[0]-start>=a.seconds-.1,'Simulation did not advance'
for k in rows:assert rows[k],f'Missing {k}'
data={k:np.array(v) for k,v in rows.items()}
def stats(v):return {'min':float(np.min(v)),'max':float(np.max(v)),'peak_to_peak':float(np.ptp(v)),'std':float(np.std(v)),'end_minus_start':float(v[-1]-v[0])}
report={'duration_sim_s':now[0]-start,'counts':{k:len(v) for k,v in rows.items()},
 'imu_acc_norm_m_s2':stats(np.linalg.norm(data['imu'][:,1:4],axis=1)),
 'lio_z_m':stats(data['lio'][:,3]),'robot_z_m':stats(data['robot'][:,1]),
 'truth_z_m':stats(data['truth'][:,3]),'lift_m':stats(data['lift'][:,1]),
 'scope':'Stationary height variation; Gazebo truth used only for evaluation, never for pose correction'}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(a.output.with_suffix('.npz'),**data);print(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown()
