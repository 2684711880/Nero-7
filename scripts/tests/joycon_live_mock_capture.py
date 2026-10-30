#!/usr/bin/env python3
"""Real Joy-Cons + fake ROS drivers in isolated domain; never opens CAN."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Empty, SetBool

assert os.environ.get('ROS_DOMAIN_ID') == '105', 'requires isolated ROS_DOMAIN_ID=105'
root = Path(__file__).resolve().parents[2]
out = root / 'hardware_ws/validation/joycon_timing/live_mock'
out.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('joycon_fake_dual_driver')
commands = {'left': [], 'right': []}
gates = {'left': [], 'right': []}
stops = {'left': 0, 'right': 0}
publishers = {}
for side in ('left', 'right'):
    publishers[side] = (node.create_publisher(PoseStamped, f'/{side}/feedback/tcp_pose', 10),
                        node.create_publisher(JointState, f'/{side}/feedback/joint_states', 10))
    def gate(req, resp, side=side):
        gates[side].append(req.data); resp.success = True; return resp
    def stop(req, resp, side=side):
        stops[side] += 1; return resp
    def params(req, resp, side=side):
        resp.values = [ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=100),
                       ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='can1' if side=='left' else 'can0')]
        return resp
    node.create_service(SetBool, f'/{side}/control_enable', gate)
    node.create_service(Empty, f'/{side}/emergency_stop', stop)
    node.create_service(GetParameters, f'/{side}/agx_arm_ctrl_single_node/get_parameters', params)
    node.create_subscription(PoseStamped, f'/{side}/control/move_p',
                             lambda msg, s=side: commands[s].append(time.monotonic()), 10)

def publish():
    for side, (pose_pub, joints_pub) in publishers.items():
        pose = PoseStamped(); pose.header.stamp = node.get_clock().now().to_msg(); pose.pose.orientation.w = 1.
        pose_pub.publish(pose)
        joints = JointState(); joints.header.stamp = node.get_clock().now().to_msg()
        joints.name = [f'joint{i}' for i in range(1,8)] + ['gripper']
        joints.position = [0.]*7 + [.04]; joints.velocity = [0.]*8
        joints_pub.publish(joints)
node.create_timer(.005, publish)
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()
log = (out/'teleop.log').open('w')
child = subprocess.Popen([sys.executable, '-u', str(root / 'scripts/joycon_move_p_dual.py'), '--execute'],
                         stdout=log, stderr=subprocess.STDOUT)
try:
    deadline = time.monotonic()+60
    enough_since = None
    while time.monotonic()<deadline and child.poll() is None:
        if all(len(commands[side]) >= 200 for side in ('left', 'right')):
            if enough_since is None:
                enough_since = time.monotonic()
            elif time.monotonic() - enough_since >= 5:
                break
        time.sleep(.1)
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
    child.wait(timeout=10)
    report = {'hardware_motion_sent': False, 'ros_domain': 105, 'teleop_exit_code': child.returncode,
              'gate_calls': gates, 'stop_requests': stops, 'commands': {}}
    for side, timestamps in commands.items():
        gaps = np.diff(timestamps)
        report['commands'][side] = {'count': len(timestamps),
            'p95_gap_ms': round(float(np.percentile(gaps,95))*1000,2) if len(gaps) else None,
            'max_gap_ms': round(float(max(gaps))*1000,2) if len(gaps) else None}
    (out/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
finally:
    if child.poll() is None: child.kill(); child.wait()
    log.close()
    if rclpy.ok(): rclpy.shutdown()
    thread.join(timeout=3)
    node.destroy_node()
