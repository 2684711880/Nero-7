#!/usr/bin/env python3
"""Read-only physical acceptance for the bottle task; does not command motion."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
parser.add_argument('--task-id', default='bottle_transfer_004')
parser.add_argument('--timeout', type=float, default=300)
parser.add_argument('--return-home', action='store_true')
args = parser.parse_args()
rclpy.init()
node = Node('task_bottle_acceptance_observer')
samples, events = [], []
terminal = None
placement_active = False
placement_joints = []
last_left_j1 = None
last_left_j2 = None
initial_left_j1 = None
last = 0.

def models(msg):
    global last
    if time.monotonic()-last < .1:
        return
    last = time.monotonic()
    row = {'wall_time': last}
    for name in ('water_bottle', 'mobile_dual_nero'):
        if name in msg.name:
            i = msg.name.index(name)
            p = msg.pose[i]
            row[name] = {'xyz': [p.position.x, p.position.y, p.position.z],
                         'q': [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}
    samples.append(row)

def status(msg):
    global terminal, placement_active
    data = json.loads(msg.data)
    if data.get('task_id') != args.task_id:
        return
    events.append(data)
    placement_active = data.get('actions', {}).get('r7', {}).get('status') == 'RUNNING'
    if data['status'] in ('SUCCEEDED', 'FAILED', 'CANCELED', 'STOP_UNCONFIRMED'):
        terminal = data

def joints(msg):
    global last_left_j1, last_left_j2, initial_left_j1
    all_values = dict(zip(msg.name, msg.position))
    if 'left_joint1' in all_values:
        last_left_j1 = all_values['left_joint1']
        if initial_left_j1 is None:
            initial_left_j1 = last_left_j1
    if 'left_joint2' in all_values:
        last_left_j2 = all_values['left_joint2']
    if placement_active:
        values = dict(zip(msg.name, msg.position))
        names = ['right_joint'+str(i) for i in range(1, 8)]
        if all(name in values for name in names):
            placement_joints.append([values[name] for name in names])

node.create_subscription(JointState, '/joint_states', joints, 10)
node.create_subscription(ModelStates, '/gazebo/model_states', models, 10)
qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String, '/robot_task/status', status, qos)
deadline = time.monotonic()+args.timeout
try:
    while terminal is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    until = time.monotonic()+3
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
    bottle = samples[-1]['water_bottle'] if samples else None
    metrics = {}
    if bottle:
        x,y,z = bottle['xyz']
        q = bottle['q']
        metrics = {'final_xyz':bottle['xyz'], 'xy_error_m':math.hypot(x,y-1.6),
                   'height_error_m':abs(z-.72), 'upright_cos':1-2*(q[0]**2+q[1]**2)}
    passed = bool(terminal and terminal['status']=='SUCCEEDED' and metrics and
        metrics['xy_error_m'] < .04 and metrics['height_error_m'] < .02 and metrics['upright_cos'] > .97)
    if placement_joints:
        spans = [max(row[i] for row in placement_joints)-min(row[i] for row in placement_joints) for i in range(7)]
        metrics['placement_joint_spans_rad'] = spans
        metrics['placement_max_joint_span_rad'] = max(spans)
        passed = passed and max(spans) < 1.5
    else:
        passed = False
    if args.return_home:
        base = samples[-1].get('mobile_dual_nero') if samples else None
        if base is None or last_left_j1 is None or last_left_j2 is None:
            passed = False
        else:
            x,y,_ = base['xyz']
            qx,qy,qz,qw = base['q']
            yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            metrics.update(base_final_xyz=base['xyz'], base_home_xy_error_m=math.hypot(x,y),
                base_home_yaw_error_rad=abs(yaw), left_j1_rad=last_left_j1,
                left_j1_change_rad=abs(last_left_j1-initial_left_j1), left_j2_rad=last_left_j2,
                left_j2_target_error_rad=abs(last_left_j2-math.pi/3))
            passed = passed and metrics['base_home_xy_error_m'] < .03 and abs(yaw) < .04 and metrics['left_j2_target_error_rad'] < .02 and metrics['left_j1_change_rad'] < .02
    report = {'passed':passed, 'task_id':args.task_id, 'metrics':metrics,
              'task_status':terminal, 'grasp_model':'checked_fixed_joint',
              'hardware_connected':False, 'samples':samples, 'events':events, 'placement_joints':placement_joints}
    Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'passed':passed, 'metrics':metrics, 'status':terminal['status'] if terminal else 'TIMEOUT'}))
finally:
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if passed else 1)
