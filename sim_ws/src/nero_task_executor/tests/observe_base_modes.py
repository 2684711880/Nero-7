#!/usr/bin/env python3
"""Read-only Gazebo acceptance: prove actual translation/rotation overlap."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
parser.add_argument('--task-id', default='base_modes_001')
parser.add_argument('--timeout', type=float, default=240)
args = parser.parse_args()
rclpy.init()
node = Node('observe_cpp_base_modes', parameter_overrides=[Parameter('use_sim_time', value=True)])
events, samples = [], []
terminal = None
phase = None
last = 0.
command = [0.,0.,0.]


def status(msg):
    global terminal, phase
    state = json.loads(msg.data)
    if state.get('task_id') != args.task_id:
        return
    events.append(state)
    phase = next((key for key, item in state.get('actions', {}).items() if item['status']=='RUNNING'), None)
    if state['status'] in ('SUCCEEDED','FAILED','CANCELED','STOP_UNCONFIRMED','REJECTED'):
        terminal = state


def velocity(msg):
    global command
    command = [msg.linear.x,msg.linear.y,msg.angular.z]


def models(msg):
    global last
    now = node.get_clock().now().nanoseconds*1e-9
    if now-last<.05 or 'mobile_dual_nero' not in msg.name:
        return
    last = now
    index = msg.name.index('mobile_dual_nero')
    p,v = msg.pose[index],msg.twist[index]
    q = p.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    samples.append({'sim_time':now, 'phase':phase,'pose':[p.position.x,p.position.y,yaw],
        'world_velocity':[v.linear.x,v.linear.y,v.angular.z], 'command':list(command)})

qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,'/robot_task/status',status,qos)
node.create_subscription(Twist,'/cmd_vel',velocity,10)
node.create_subscription(ModelStates,'/gazebo/model_states',models,qos_profile_sensor_data)
end = time.monotonic()+args.timeout
try:
    while terminal is None and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
    settle = time.monotonic()+2
    while time.monotonic()<settle:
        rclpy.spin_once(node,timeout_sec=.1)
    translation = [row for row in samples if row['phase']=='translate']
    rotation = [row for row in samples if row['phase']=='rotate']
    combined = [row for row in samples if row['phase']=='combined']
    metrics = {}
    if translation:
        baseline = translation[0]['pose'][2]
        metrics['translation_heading_excursion_rad'] = max(abs(math.atan2(math.sin(x['pose'][2]-baseline),math.cos(x['pose'][2]-baseline))) for x in translation)
    if rotation:
        x,y,_ = rotation[0]['pose']
        metrics['rotation_position_excursion_m'] = max(math.hypot(row['pose'][0]-x,row['pose'][1]-y) for row in rotation)
    overlap = [row for row in combined if math.hypot(*row['world_velocity'][:2])>.01 and abs(row['world_velocity'][2])>.03]
    # Sum measured intervals, not just nonzero commands or a start/end pose.
    duration = 0.
    for previous,current in zip(combined,combined[1:]):
        v = previous['world_velocity']
        if math.hypot(*v[:2])>.01 and abs(v[2])>.03:
            duration += min(.2,max(0.,current['sim_time']-previous['sim_time']))
    metrics['actual_overlap_duration_s'] = duration
    metrics['actual_overlap_samples'] = len(overlap)
    if samples:
        x,y,yaw = samples[-1]['pose']
        metrics.update(final_pose=[x,y,yaw],final_position_error_m=math.hypot(x,y),final_heading_error_rad=abs(yaw))
    passed = bool(terminal and terminal['status']=='SUCCEEDED' and translation and rotation and combined and
        metrics['translation_heading_excursion_rad']<.04 and metrics['rotation_position_excursion_m']<.04 and
        duration>.5 and metrics['final_position_error_m']<.03 and metrics['final_heading_error_rad']<.04)
    report = {'passed':passed,'task_status':terminal,'metrics':metrics,'samples':samples,'events':events,
              'pose_source':'independent Gazebo ModelStates truth','hardware_connected':False}
    Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'passed':passed,'metrics':metrics,'status':terminal['status'] if terminal else 'TIMEOUT'}))
finally:
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if passed else 1)
