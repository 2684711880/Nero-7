#!/usr/bin/env python3
"""Bounded simulation-only LIO run: optional slow base motion; export map/evidence."""
import argparse
import csv
import json
import math
import struct
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg
from moveit_msgs.srv import GetStateValidity
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, PointCloud2
from std_msgs.msg import String
from mobile_dual_nero.mid360 import mount_pose, moves_with_upper_body

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--algorithm', choices=['superlio','fastlio2'], default='superlio')
parser.add_argument('--mount', required=True, choices=['chassis_up','chest_down','head_tilt'])
parser.add_argument('--xyz', default='', help='Same base_link xyz override as the launch')
parser.add_argument('--tilt-deg', default='', help='Same head pitch degrees as launch')
parser.add_argument('--rpy', default='', help='Same base_link rpy override as the launch')
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--no-ceiling', action='store_true', help='Match launch ceiling:=false')
parser.add_argument('--move', action='store_true', help='Drive backwards slowly for 6 simulated seconds')
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('verify_store_mapping', parameter_overrides=[Parameter('use_sim_time',value=True)])
counts={};latest={};samples=[];stats=[];voxels={};bad_stamps=0;bad_offsets=0;last_stamps={}
cloud_counts=[];imu_vectors=[];clock_times={};subscriptions=[]

def rot(q):
 x,y,z,w=q.x,q.y,q.z,q.w
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
 [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
 [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def recv(key,msg):
 global bad_stamps,bad_offsets
 latest[key]=msg;counts[key]=counts.get(key,0)+1
 if hasattr(msg,'header'):
  stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
  if key in last_stamps and stamp<=last_stamps[key]:bad_stamps+=1
  last_stamps[key]=stamp
  clock_times.setdefault(key,[]).append(stamp)
 if key=='raw':
  cloud_counts.append(msg.point_num)
  offsets=[p.offset_time for p in msg.points]
  if any(b<a for a,b in zip(offsets,offsets[1:])) or (offsets and offsets[-1]>110000000):bad_offsets+=1
 if key=='imu':imu_vectors.append([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
 if key=='stats':stats.append(json.loads(msg.data))
 if key=='map':
  # The algorithm publishes deskewed frame clouds, accumulated here at 5 cm.
  fields={f.name:f.offset for f in msg.fields}
  for row in range(msg.height):
   for column in range(msg.width):
    offset=row*msg.row_step+column*msg.point_step
    p=tuple(struct.unpack_from('<f',msg.data,offset+fields[k])[0] for k in ('x','y','z'))
    if all(math.isfinite(v) for v in p):voxels[tuple(int(math.floor(v/.05)) for v in p)]=p

for typ,topic,key,sensor in [
 (CustomMsg,'/mid360/lidar','raw',True),(Imu,'/mid360/imu','imu',True),
 (String,'/mid360/diagnostics','stats',False),(Odometry,'/lio/odom','lio',False),
 (ModelStates,'/gazebo/model_states','truth',False),
 (PointCloud2,'/lio/cloud_world','map',False),(JointState,'/joint_states','joints',False)]:
 subscriptions.append(node.create_subscription(typ,topic,lambda m,k=key:recv(k,m),qos_profile_sensor_data if sensor else 10))
cmd=node.create_publisher(Twist,'/cmd_vel',10)
service=node.create_client(GetStateValidity,'/check_state_validity')
assert service.wait_for_service(timeout_sec=30), 'MoveIt not ready'
wall_end=time.monotonic()+40
while not all(k in latest for k in ('raw','imu','lio','truth','joints','map')):
 if time.monotonic()>wall_end:raise RuntimeError(f'Missing data: {counts}')
 rclpy.spin_once(node,timeout_sec=.1)
# This script is restricted to the expected simulated entity and sensor stream.
assert 'mobile_dual_nero' in latest['truth'].name and latest['raw'].header.frame_id=='mid360_link'
request=GetStateValidity.Request();request.robot_state.joint_state=latest['joints'];request.robot_state.is_diff=True
future=service.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
initial_collision_free=future.done() and future.result().valid
if not initial_collision_free:raise RuntimeError(f'Initial collision: {future.result()}')
xyz,_=mount_pose(args.mount,args.xyz,args.rpy,args.tilt_deg)
base0=latest['truth'].pose[latest['truth'].name.index('mobile_dual_nero')]
base0=np.array([base0.position.x,base0.position.y,base0.position.z])
# Use the received sensor stamp for timing; reject stale startup clocks.
start=latest['raw'].header.stamp.sec+latest['raw'].header.stamp.nanosec*1e-9
print('capture_start',start, 'node_clock',node.get_clock().now().nanoseconds*1e-9,flush=True)
assert start > 0, 'No simulation clock'
next_sample=start;wall_end=time.monotonic()+90
try:
 while True:
  rclpy.spin_once(node,timeout_sec=.02)
  now=latest['raw'].header.stamp.sec+latest['raw'].header.stamp.nanosec*1e-9;elapsed=now-start
  if time.monotonic()>wall_end:raise RuntimeError('Simulation stopped or too slow')
  if elapsed>=14:break
  twist=Twist()
  if args.move and 3<elapsed<9:twist.linear.x=-.08
  cmd.publish(twist)
  if now<next_sample:continue
  next_sample=now+.1
  truth=latest['truth'].pose[latest['truth'].name.index('mobile_dual_nero')]
  sensor_xyz=np.array(xyz,dtype=float)
  if moves_with_upper_body(args.mount):
   sensor_xyz[2]+=dict(zip(latest['joints'].name,latest['joints'].position)).get('upper_body_lift_joint',0.)
  sensor=np.array([truth.position.x,truth.position.y,truth.position.z])+rot(truth.orientation)@sensor_xyz
  pose=latest['lio'].pose.pose
  samples.append([elapsed,*sensor,pose.position.x,pose.position.y,pose.position.z])
  distance=np.linalg.norm(np.array([truth.position.x,truth.position.y,truth.position.z])-base0)
  if distance>.9:raise RuntimeError('Bounded run exceeded 0.9 m displacement')
finally:
 for _ in range(5):cmd.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
assert samples, 'No synchronized trajectory samples'
array=np.array(samples)
# These presets preserve base x heading after gravity alignment. Translation alignment
# is used only for evaluation, never broadcast or supplied to Super-LIO.
truth=array[:,1:4]-array[0,1:4];estimated=array[:,4:7]-array[0,4:7]
errors=np.linalg.norm(estimated-truth,axis=1)
rays=sum(s['rays'] for s in stats);self_hits=sum(s['self_hits'] for s in stats)
report=dict(algorithm=args.algorithm,mount=args.mount,ceiling=not args.no_ceiling,move=args.move,initial_collision_free=initial_collision_free,
 counts=counts,nonmonotonic_stamps=bad_stamps,bad_point_offsets=bad_offsets,
 mean_returns=float(np.mean(cloud_counts)),self_return_fraction=self_hits/max(1,rays),
 mean_imu_specific_force=np.mean(imu_vectors,axis=0).tolist(),
 frequencies_hz={k:(len(v)-1)/(v[-1]-v[0]) for k,v in clock_times.items() if len(v)>1 and v[-1]>v[0]},
 truth_displacement_m=float(np.linalg.norm(truth[-1])),estimated_displacement_m=float(np.linalg.norm(estimated[-1])),
 translation_aligned_rmse_m=float(np.sqrt(np.mean(errors**2))),last_error_m=float(errors[-1]),
 map_voxels_5cm=len(voxels),pipeline_passed=bool(counts.get('lio',0)>20 and counts.get('map',0)>20 and not bad_stamps and not bad_offsets and len(voxels)>100))
# Independent collision geometry check after the chassis has stopped.
raw=latest['raw'];pose=latest['truth'].pose[latest['truth'].name.index('mobile_dual_nero')]
points_local=np.array([[p.x,p.y,p.z] for p in raw.points])
points_local=points_local[np.linalg.norm(points_local,axis=1)>2.0]
_,mount_rpy=mount_pose(args.mount,args.xyz,args.rpy,args.tilt_deg)
r,p,y=mount_rpy
Rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
Ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
Rz=np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
ray_sensor_xyz=np.array(xyz,dtype=float)
if moves_with_upper_body(args.mount):
 ray_sensor_xyz[2]+=dict(zip(latest['joints'].name,latest['joints'].position)).get('upper_body_lift_joint',0.)
points_world=(points_local@(Rz@Ry@Rx).T+ray_sensor_xyz)@rot(pose.orientation).T+np.array([pose.position.x,pose.position.y,pose.position.z])
from mobile_dual_nero.convenience_store import convenience_store_objects
surface=np.abs(points_world[:,2])
for obj in convenience_store_objects(ceiling=not args.no_ceiling):
 if not obj.get('collision',True):continue
 delta=points_world-np.array(obj['xyz'])
 if obj['kind']=='box':
  d=np.abs(delta)-np.array(obj['size'])/2
 else:
  h,radius=obj['size'];d=np.column_stack([np.linalg.norm(delta[:,:2],axis=1)-radius,np.abs(delta[:,2])-h/2])
 distance=np.abs(np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(np.max(d,axis=1),0))
 surface=np.minimum(surface,distance)
report['raw_surface_error_p95_m']=float(np.quantile(surface,.95)) if len(surface) else None
report['mapping_accuracy_passed']=bool(report['pipeline_passed'] and report['translation_aligned_rmse_m']<.10 and report['last_error_m']<.15)
(args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
(args.output/'ray_stats.json').write_text(json.dumps(stats,indent=2)+'\n')
with (args.output/'trajectory.csv').open('w') as f:
 writer=csv.writer(f);writer.writerow(['elapsed','truth_x','truth_y','truth_z','lio_x','lio_y','lio_z']);writer.writerows(samples)
points=np.asarray(list(voxels.values()),dtype=np.float32)
with (args.output/'map.pcd').open('wb') as f:
 f.write((f'# LIO accumulated output; 5 cm export voxels\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n').encode());f.write(points.tobytes())
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(12,5))
view=points[points[:,2] < 3.0-xyz[2]]  # hide roof only in the preview
axes[0].scatter(view[:,0],view[:,1],c=view[:,2],s=.4,cmap='turbo',vmin=-1,vmax=3)
axes[0].set_xlim(-7,7);axes[0].set_ylim(-6,6)
axes[0].set_aspect('equal');axes[0].set_title(f'{args.algorithm} · {args.mount} (roof hidden in preview)');axes[0].set(xlabel='x / m',ylabel='y / m')
axes[1].plot(array[:,0],errors);axes[1].set(xlabel='elapsed / s',ylabel='translation-aligned position error / m',title='Simulation truth used only for evaluation');axes[1].grid(alpha=.3)
fig.tight_layout();fig.savefig(args.output/'map_and_error.png',dpi=150);plt.close(fig)
print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()
assert report['pipeline_passed']
