#!/usr/bin/env python3
"""Real DDS + C++ base server tests with synthetic odometry, no Gazebo/hardware."""
import argparse
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nero_task_interfaces.action import ExecuteBaseMotion

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
args = parser.parse_args()
folder = Path(args.output)
folder.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = Node('base_protocol_fixture', parameter_overrides=[Parameter('use_sim_time', value=True)])
clock_pub = node.create_publisher(Clock, '/clock', 10)
odom_pub = node.create_publisher(Odometry, '/odom', 10)
task_pub = node.create_publisher(String, '/robot_task/data', 10)
cancel_pub = node.create_publisher(String, '/robot_task/cancel', 10)
pose = [0., 0., 0.]
command = [0., 0., 0.]
last_command = 0.
last_tick = start = time.monotonic()
odom_enabled = True
freeze = False
states = []
trace = []
processes = []
logs = []
checks = []


def on_command(msg):
    global command, last_command
    command = [msg.linear.x, msg.linear.y, msg.angular.z]
    last_command = time.monotonic()


def fixture_tick():
    global last_tick
    now = time.monotonic()
    dt = min(.05, now-last_tick)
    last_tick = now
    v = command if now-last_command < .5 else [0., 0., 0.]
    if not freeze:
        c,s = math.cos(pose[2]),math.sin(pose[2])
        pose[0] += (c*v[0]-s*v[1])*dt
        pose[1] += (s*v[0]+c*v[1])*dt
        pose[2] = math.atan2(math.sin(pose[2]+v[2]*dt), math.cos(pose[2]+v[2]*dt))
    seconds = 10.+now-start
    tick = Clock()
    tick.clock.sec = int(seconds)
    tick.clock.nanosec = int((seconds-int(seconds))*1e9)
    clock_pub.publish(tick)
    if odom_enabled:
        msg = Odometry()
        msg.header.stamp = tick.clock
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        if not freeze:
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = v
        odom_pub.publish(msg)
    trace.append({'t': now-start, 'pose': list(pose), 'command': list(v)})


node.create_subscription(Twist, '/cmd_vel', on_command, 10)
qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String, '/robot_task/status', lambda m: states.append(json.loads(m.data)), qos)
node.create_timer(.02, fixture_tick, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))


def wait(predicate, timeout=30):
    end = time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>end:
            raise TimeoutError('Missing expected base protocol outcome')
        rclpy.spin_once(node, timeout_sec=.02)


def launch(command, name):
    log = (folder/(name+'.log')).open('w')
    logs.append(log)
    p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(p)
    return p


def action(a, kind, parameters, timeout=60):
    return {'action_id':a, 'type':kind, 'parameters':dict(frame_id='odom', **parameters),
            'wait_for':[], 'timeout_s':timeout}


def send(key, actions):
    task = {'version':1, 'task_id':key, 'base_actions':actions, 'left_arm_actions':[], 'right_arm_actions':[]}
    task_pub.publish(String(data=json.dumps(task)))


def latest(key):
    return next((x for x in reversed(states) if x.get('task_id')==key), {})

try:
    with tempfile.TemporaryDirectory(prefix='nero-base-protocol-') as temporary:
        launch(['ros2','run','nero_task_executor','sim_base_server','--ros-args','-p','use_sim_time:=true'], 'server')
        launch(['ros2','run','nero_task_executor','task_executor','--ros-args','-p','use_sim_time:=true',
                '-p','backend:=simulation','-p','state_file:='+temporary+'/state.json'], 'executor')
        direct = ActionClient(node, ExecuteBaseMotion, '/task_motion/base')
        wait(lambda: direct.server_is_ready() and task_pub.get_subscription_count()>0)
        wait(lambda: time.monotonic()-last_command>.05, timeout=2)
        send('three_modes', [action('t','base_translate',{'position_xy_m':[.06,0]}),
            action('r','base_rotate',{'yaw_rad':.35}),
            action('c','base_move_and_rotate',{'position_xy_m':[0,0],'yaw_rad':0})])
        wait(lambda: latest('three_modes').get('status') in ('SUCCEEDED','FAILED','STOP_UNCONFIRMED'), timeout=45)
        assert latest('three_modes')['status']=='SUCCEEDED', latest('three_modes')
        checks.append('All three action types execute through C++ and complete through the Python scheduler')
        send('mixed', [action('old','base_move_to',{'position_xy_m':[.04,0],'yaw_rad':0}),
            action('new','base_rotate',{'yaw_rad':.2}),
            action('old_again','base_move_to',{'position_xy_m':[0,0],'yaw_rad':0})])
        wait(lambda: latest('mixed').get('status') in ('SUCCEEDED','FAILED','STOP_UNCONFIRMED'), timeout=25)
        assert latest('mixed')['status']=='SUCCEEDED', latest('mixed')
        checks.append('Legacy Python and new C++ base actions hand over ownership within one lane')
        send('cancel', [action('c','base_move_and_rotate',{'position_xy_m':[.4,0],'yaw_rad':1.57}),
                        action('blocked','base_rotate',{'yaw_rad':0})])
        wait(lambda: latest('cancel').get('actions',{}).get('c',{}).get('status')=='RUNNING')
        wait(lambda: math.hypot(*command[:2])>.005 or abs(command[2])>.01)
        cancel_pub.publish(String(data='cancel'))
        wait(lambda: latest('cancel').get('status') in ('CANCELED','STOP_UNCONFIRMED'), timeout=12)
        assert latest('cancel')['status']=='CANCELED', latest('cancel')
        assert latest('cancel')['actions']['blocked']['status']=='BLOCKED'
        assert math.hypot(*command[:2])<1e-9 and abs(command[2])<1e-9
        checks.append('Cancel confirms stop and blocks the next action')
        freeze = True
        send('timeout', [action('t','base_translate',{'position_xy_m':[pose[0]+.04,pose[1]]}, timeout=2)])
        wait(lambda: latest('timeout').get('status') in ('FAILED','STOP_UNCONFIRMED'), timeout=12)
        assert latest('timeout')['status']=='FAILED', latest('timeout')
        checks.append('Action timeout cancels the C++ command and confirms synthetic stop')
        freeze = False
        send('stale', [action('t','base_translate',{'position_xy_m':[pose[0]+.3,pose[1]]})])
        wait(lambda: latest('stale').get('actions',{}).get('t',{}).get('status')=='RUNNING')
        wait(lambda: math.hypot(*command[:2])>.005)
        odom_enabled = False
        wait(lambda: latest('stale').get('status')=='STOP_UNCONFIRMED', timeout=12)
        assert math.hypot(*command[:2])<1e-9 and abs(command[2])<1e-9
        checks.append('Stale odometry commands zero and reports stop unconfirmed rather than success')
        odom_enabled = True
        send('blocked_after_stale', [action('t','base_translate',{'position_xy_m':[0,0]})])
        wait(lambda: latest('blocked_after_stale').get('status')=='REJECTED')
        request = action('direct','base_rotate',{'yaw_rad':0})
        future = direct.send_goal_async(ExecuteBaseMotion.Goal(command_json=json.dumps(request)))
        wait(future.done)
        assert not future.result().accepted
        checks.append('Both scheduler ownership and the C++ server latch reject new motion after unknown stop')
        report = {'passed':True, 'checks':checks, 'odometry':'synthetic kinematic fixture',
                  'hardware_connected':False, 'gazebo_started':False}
        (folder/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (folder/'events.json').write_text(json.dumps({'states':states,'trace':trace},indent=2)+'\n')
        print(json.dumps(report))
finally:
    for p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                p.wait(timeout=5)
    for log in logs:log.close()
    node.destroy_node()
    rclpy.shutdown()
