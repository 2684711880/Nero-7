#!/usr/bin/env python3
"""Take one newly received real-robot ROS feedback sample; never command hardware."""
import json
import math
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

names = [f'right_joint{i}' for i in range(1, 8)]
rclpy.init()
node = Node('sim_hole_initial_feedback')
sample = {}

def receive(msg, dedicated=False):
    values = dict(zip(msg.name, msg.position))
    if dedicated and all(f'joint{i}' in values for i in range(1, 8)):
        values = {name: values[f'joint{i}'] for i, name in enumerate(names, 1)}
    if all(name in values and math.isfinite(values[name]) for name in names):
        sample.update({name: float(values[name]) for name in names})

subscriptions = [node.create_subscription(JointState, '/right/feedback/joint_states', lambda m: receive(m, True), 10)]
deadline = time.monotonic() + 2.0
while not sample and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
if sample:
    with open(sys.argv[1], 'w') as f:
        json.dump(sample, f)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if sample else 1)
