#!/usr/bin/env python3
"""Read-only timing/steering/drive correlation; never publishes robot commands."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
from ranger_air_gazebo.kinematics import WHEELS, inverse, wrap


parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
parser.add_argument('--task-id', default='bottle_transfer_combined_001')
parser.add_argument('--timeout', type=float, default=300)
args = parser.parse_args()
rclpy.init()
node = Node('base_smoothness_observer')
rows, events = [], []
phase = terminal = None
sim_time = 0.
command = [0., 0., 0.]
angles = steering = [0.] * 4
speed = [0., 0.]
pose = [0., 0., 0.]
last_clock_wall = last_cmd_wall = last_joint_wall = 0.


def clock(msg):
    global sim_time, last_clock_wall
    sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
    last_clock_wall = time.monotonic()


def cmd(msg):
    global command, last_cmd_wall
    command = [msg.linear.x, msg.linear.y, msg.angular.z]
    last_cmd_wall = time.monotonic()


def joints(msg):
    global angles, last_joint_wall
    values = dict(zip(msg.name, msg.position))
    if all(w+'_steering_joint' in values for w in WHEELS):
        angles = [values[w+'_steering_joint'] for w in WHEELS]
        last_joint_wall = time.monotonic()


def steer(msg):
    global steering
    steering = list(msg.data)


def models(msg):
    global speed, pose
    if 'mobile_dual_nero' in msg.name:
        i = msg.name.index('mobile_dual_nero')
        p, v = msg.pose[i], msg.twist[i]
        q = p.orientation
        speed = [math.hypot(v.linear.x, v.linear.y), abs(v.angular.z)]
        pose = [p.position.x, p.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


def state(msg):
    global phase, terminal
    s = json.loads(msg.data)
    if s.get('task_id') != args.task_id:
        return
    events.append(s)
    phase = next((k for k, a in s.get('actions', {}).items()
                  if a.get('lane') == 'base' and a['status'] == 'RUNNING'), None)
    if s['status'] in ('SUCCEEDED', 'FAILED', 'CANCELED', 'STOP_UNCONFIRMED', 'REJECTED'):
        terminal = s


def wheels(msg):
    now = time.monotonic()
    desired, rates = inverse(*command, angles)
    peak = max(abs(v) for v in rates)
    rows.append({'wall': now, 'sim': sim_time, 'phase': phase, 'cmd': list(command),
                 'angles': list(angles), 'steering': list(steering), 'wheels': list(msg.data),
                 'desired_error': max(abs(wrap(a-b)) for a,b in zip(desired, angles)),
                 'weighted_error': max(abs(wrap(a-b))*abs(v)/peak for a,b,v in zip(desired,angles,rates)) if peak>1e-6 else 0.,
                 'actual_speed': list(speed), 'pose': list(pose),
                 'cmd_age': now-last_cmd_wall, 'joint_age': now-last_joint_wall,
                 'clock_age': now-last_clock_wall})


node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
node.create_subscription(Twist, '/cmd_vel', cmd, 100)
node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)
node.create_subscription(ModelStates, '/gazebo/model_states', models, qos_profile_sensor_data)
node.create_subscription(Float64MultiArray, '/steering_controller/commands', steer, 100)
node.create_subscription(Float64MultiArray, '/wheel_controller/commands', wheels, 100)
qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String, '/robot_task/status', state, qos)
end = time.monotonic()+args.timeout
try:
    while terminal is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    until = time.monotonic()+1
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
    metrics = {}
    for name in sorted({r['phase'] for r in rows if r['phase']}):
        all_rows = [r for r in rows if r['phase'] == name]
        # Exclude start/arrival transients: middle 60% of each base action.
        a,b = all_rows[0]['sim'], all_rows[-1]['sim']
        cruise = [r for r in all_rows if a+.2*(b-a) < r['sim'] < a+.8*(b-a)]
        active = [r for r in cruise if math.hypot(*r['cmd'][:2])>.01 and abs(r['cmd'][2])>.03]
        blocked = [r for r in active if max(abs(v) for v in r['wheels']) < 1e-6]
        states = [max(abs(v) for v in r['wheels'])<1e-6 for r in active]
        metrics[name] = {'sim_duration_s': b-a, 'wall_duration_s': all_rows[-1]['wall']-all_rows[0]['wall'],
                         'cruise_combined_samples': len(active),
                         'cruise_blocked_fraction': len(blocked)/max(1,len(active)),
                         'cruise_stop_entries': sum(not a and b for a,b in zip(states,states[1:])),
                         'blocked_with_large_steering_error_fraction': sum(r['desired_error']>.12 for r in blocked)/max(1,len(blocked)),
                         'cruise_near_zero_actual_fraction': sum(r['actual_speed'][0]<.003 and r['actual_speed'][1]<.003 for r in cruise)/max(1,len(cruise)),
                         'max_cmd_age_s': max(r['cmd_age'] for r in all_rows),
                         'max_joint_age_s': max(r['joint_age'] for r in all_rows)}
    report = {'task_status': terminal, 'metrics': metrics, 'samples': rows,
              'events': events, 'hardware_connected': False,
              'desired_error_note': 'Inverse of latest cmd_vel and measured angles; controller additionally filters cmd_vel'}
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, separators=(',', ':'))+'\n')
    print(json.dumps({'status': terminal['status'] if terminal else 'TIMEOUT', 'metrics': metrics}), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if terminal and terminal['status']=='SUCCEEDED' else 1)
