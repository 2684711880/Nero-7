#!/usr/bin/env python3
"""Read-only Gazebo/MoveIt acceptance trace; not a task controller."""
import json
import sys
import time
from pathlib import Path
import rclpy
from gazebo_msgs.msg import ModelStates
from moveit_msgs.msg import PlanningScene
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('pick_place_observer')
rows, events, attachments = [], [], []
last = 0.
def update(m):
    global last
    now = time.monotonic()
    if now-last < .05:
        return
    last = now
    row = {'t': now}
    for name in ('mobile_dual_nero', 'water_bottle'):
        if name in m.name:
            i = m.name.index(name)
            p, v = m.pose[i], m.twist[i]
            row[name] = {'xyz': [p.position.x, p.position.y, p.position.z],
                         'q': [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
                         'v': [v.linear.x, v.linear.y, v.linear.z]}
    rows.append(row)
def planning(m):
    if m.robot_state.attached_collision_objects:
        attachments.append({'t': time.monotonic(), 'objects': [
            {'id': a.object.id, 'operation': a.object.operation}
            for a in m.robot_state.attached_collision_objects]})
node.create_subscription(ModelStates, '/gazebo/model_states', update, 10)
node.create_subscription(PlanningScene, '/planning_scene', planning, 10)
node.create_subscription(PlanningScene, '/monitored_planning_scene', planning, 10)
node.create_subscription(String, '/right/control/result', lambda m: events.append(
    {'t': time.monotonic(), 'result': m.data}), 10)
try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=.1)
except KeyboardInterrupt:
    pass
finally:
    Path(sys.argv[1]).write_text(json.dumps({'samples': rows, 'commands': events, 'attachments': attachments}, indent=2))
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
