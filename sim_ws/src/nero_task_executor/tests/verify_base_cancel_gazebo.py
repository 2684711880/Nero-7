#!/usr/bin/env python3
"""Cancel a real Gazebo combined motion after measured translation and turning begin."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

parser=argparse.ArgumentParser()
parser.add_argument('--output',required=True)
args=parser.parse_args()
rclpy.init();node=Node('verify_base_cancel_gazebo')
pose=None;speed=None;state=None;events=[]
key='base_cancel_gazebo_001'
def models(m):
    global pose,speed
    if 'mobile_dual_nero' not in m.name:return
    i=m.name.index('mobile_dual_nero');p,v=m.pose[i],m.twist[i]
    pose=[p.position.x,p.position.y]
    speed=[math.hypot(v.linear.x,v.linear.y),abs(v.angular.z)]
def status(m):
    global state
    s=json.loads(m.data)
    if s.get('task_id')==key:state=s;events.append(s)
qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(ModelStates,'/gazebo/model_states',models,qos_profile_sensor_data)
node.create_subscription(String,'/robot_task/status',status,qos)
pub=node.create_publisher(String,'/robot_task/data',10)
cancel=node.create_publisher(String,'/robot_task/cancel',10)
def wait(predicate,timeout):
    end=time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>end:raise TimeoutError('Gazebo cancel verification timed out')
        rclpy.spin_once(node,timeout_sec=.02)
try:
    wait(lambda:pub.get_subscription_count()>0 and pose is not None,10)
    task={'version':1,'task_id':key,'left_arm_actions':[],'right_arm_actions':[],
          'base_actions':[{'action_id':'moving','type':'base_move_and_rotate',
              'parameters':{'frame_id':'odom','position_xy_m':[.3,.15],'yaw_rad':.9},'wait_for':[],'timeout_s':45},
              {'action_id':'blocked','type':'base_rotate','parameters':{'frame_id':'odom','yaw_rad':0},'wait_for':[],'timeout_s':45}]}
    pub.publish(String(data=json.dumps(task)))
    wait(lambda:state is not None and state['status']=='RUNNING' and speed[0]>.01 and speed[1]>.03,25)
    at_cancel=list(pose);moving_speed=list(speed)
    cancel.publish(String(data=key))
    wait(lambda:state['status'] in ('CANCELED','STOP_UNCONFIRMED','FAILED'),12)
    until=time.monotonic()+2
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
    passed=state['status']=='CANCELED' and state['actions']['blocked']['status']=='BLOCKED' and speed[0]<.015 and speed[1]<.02
    report={'passed':passed,'task_status':state,'actual_speed_when_canceled':moving_speed,
        'actual_final_speed':speed,'actual_xy_after_braking':pose,
        'braking_displacement_m':math.hypot(pose[0]-at_cancel[0],pose[1]-at_cancel[1]),
        'pose_source':'Gazebo ModelStates','hardware_connected':False,'events':events}
    Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('events','task_status')}))
finally:
    node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if passed else 1)
