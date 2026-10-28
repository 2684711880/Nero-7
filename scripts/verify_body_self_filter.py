#!/usr/bin/env python3
"""Read-only live check: exact Livox metadata/subsequence preservation and LIO output."""
import argparse
from collections import deque
import json
from pathlib import Path
import time
import rclpy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
p.add_argument('--require-removal',action='store_true',help='Require self returns in this particular test pose')
a=p.parse_args()
rclpy.init();n=rclpy.create_node('verify_body_filter');raw={};filtered={};stats=[];odom=[];sensor=[]
def key(m):return (m.header.stamp.sec,m.header.stamp.nanosec)
def raw_cb(m):
 raw[key(m)]=m
 while len(raw)>12:raw.pop(next(iter(raw)))
def filtered_cb(m):
 filtered[key(m)]=m
 while len(filtered)>12:filtered.pop(next(iter(filtered)))
subs=[n.create_subscription(CustomMsg,'/mid360/lidar',raw_cb,10),
 n.create_subscription(CustomMsg,'/mid360/lidar_filtered',filtered_cb,10),
 n.create_subscription(String,'/mid360/self_filter/diagnostics',lambda m:stats.append(json.loads(m.data)),10),
 n.create_subscription(Odometry,'/lio/odom',lambda m:odom.append(key(m)),10),
 n.create_subscription(String,'/mid360/diagnostics',lambda m:sensor.append(json.loads(m.data)),10)]
end=time.monotonic()+35;checked=0
while time.monotonic()<end and (checked<8 or len(odom)<5):
 rclpy.spin_once(n,timeout_sec=.01)
 for stamp in list(raw.keys()&filtered.keys()):
  src,out=raw.pop(stamp),filtered.pop(stamp)
  assert out.point_num==len(out.points)<=src.point_num
  for field in src.get_fields_and_field_types():
   if field not in ('points','point_num'):
    left,right=getattr(src,field),getattr(out,field)
    assert (list(left)==list(right) if field=='rsvd' else left==right),field
  it=iter(src.points)
  for point in out.points:
   assert any(candidate==point for candidate in it),'Output is not an unchanged ordered input subsequence'
  checked+=1
assert checked>=8 and len(odom)>=5, (checked,len(odom))
assert stats and min(r['near_1m_kept'] for r in stats)>0
if a.require_removal:assert sum(r['removed'] for r in stats)>0,'No self points encountered in this pose'
report={'matched_scans':checked,'removal_required':a.require_removal,'all_livox_metadata_and_retained_points_unchanged':True,
 'lio_odom_messages':len(odom),'mean_removed_fraction':sum(r['removed']/r['input'] for r in stats)/len(stats),
 'mean_near_1m_kept_per_scan':sum(r['near_1m_kept'] for r in stats)/len(stats),
 'sensor_self_return_fraction':sum(r['self_hits'] for r in sensor)/max(1,sum(r['returns'] for r in sensor)),
 'scope':'Static pipeline and preservation check; not mapping-accuracy or false-positive-rate acceptance'}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown()
