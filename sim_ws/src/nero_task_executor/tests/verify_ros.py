#!/usr/bin/env python3
"""Real DDS ingress/feedback/restart checks with mock motion only."""
import copy
import json
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT/'validation/task_executor'
OUT.mkdir(parents=True, exist_ok=True)
STATE = OUT/'ros_mock_state.json'
# Isolated validation file only; do not touch the user's default execution journal.
if STATE.exists():
    STATE.rename(OUT/f'ros_mock_state_previous_{time.time_ns()}.json')
TASK = json.loads((Path(__file__).resolve().parents[1]/'examples/bottle_transfer.json').read_text())
rclpy.init()
node = Node('verify_robot_task_transport')
qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
events = []
node.create_subscription(String, '/robot_task/status', lambda m: events.append(json.loads(m.data)), qos)
pub = node.create_publisher(String, '/robot_task/data', 10)
cancel = node.create_publisher(String, '/robot_task/cancel', 10)
process = None
logs = []
checks = []

def wait(predicate, timeout=15):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise TimeoutError('Expected ROS task status did not arrive')
        rclpy.spin_once(node, timeout_sec=.02)


def launch(index, fail=False):
    log = (OUT/f'ros_mock_{index}.log').open('w')
    logs.append(log)
    command = ['ros2','run','nero_task_executor','task_executor','--ros-args',
               '-p','state_file:='+str(STATE)]
    if fail:
        command += ['-p', 'mock_fail_actions:=[r4]']
    p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    wait(lambda: pub.get_subscription_count() > 0)
    return p


def stop(p):
    os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM)
        p.wait(timeout=5)
    assert p.returncode == 0, 'Executor did not shut down cleanly'


def send(task):
    pub.publish(String(data=json.dumps(task)))


def has(key, status):
    return any(x.get('task_id') == key and x['status'] == status for x in events)

try:
    process = launch(1)
    send(TASK)
    wait(lambda: has(TASK['task_id'], 'SUCCEEDED'))
    parallel = any(sum(a['status']=='RUNNING' for a in x.get('actions', {}).values()) == 3 for x in events)
    assert parallel
    checks.append('DDS task ingress, three concurrent lanes, successful bottle task mock')
    count = len(events)
    send(TASK)
    wait(lambda: len(events) > count)
    assert not any(x['status']=='RUNNING' for x in events[count:])
    checks.append('Duplicate completed task returns status without dispatch')
    bad = copy.deepcopy(TASK)
    bad['task_id'] = 'invalid_cycle'
    bad['base_actions'][0]['wait_for'] = ['r3']
    send(bad)
    wait(lambda: has('invalid_cycle', 'REJECTED'))
    checks.append('Cyclic dependency rejected through ROS ingress')
    active = copy.deepcopy(TASK)
    active['task_id'] = 'cancel_case'
    send(active)
    wait(lambda: has('cancel_case', 'RUNNING'))
    busy = copy.deepcopy(TASK)
    busy['task_id'] = 'busy_case'
    send(busy)
    wait(lambda: has('busy_case', 'REJECTED'))
    checks.append('Concurrent new task is rejected while robot is owned')
    cancel.publish(String(data='cancel_case'))
    wait(lambda: has('cancel_case', 'CANCELED'))
    checks.append('Cancellation returns confirmed mock terminal status')
    stop(process)
    process = None
    wait(lambda: pub.get_subscription_count()==0)
    process = launch(2, fail=True)
    count = len(events)
    send(TASK)
    wait(lambda: len(events)>count and has(TASK['task_id'], 'SUCCEEDED'))
    assert not any(x.get('task_id')==TASK['task_id'] and x['status']=='RUNNING' for x in events[count:])
    checks.append('Persistent deduplication survives executor restart')
    failed = copy.deepcopy(TASK)
    failed['task_id'] = 'grasp_failure'
    send(failed)
    wait(lambda: has('grasp_failure', 'FAILED'))
    final = [x for x in events if x.get('task_id')=='grasp_failure' and x['status']=='FAILED'][-1]
    assert final['actions']['r5']['status']=='BLOCKED'
    assert final['actions']['b2']['status']=='BLOCKED'
    checks.append('Grasp failure blocks lift and second base movement')
    interrupted = copy.deepcopy(TASK)
    interrupted['task_id'] = 'interrupted_case'
    send(interrupted)
    wait(lambda: has('interrupted_case', 'RUNNING'))
    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=5)
    process = None
    wait(lambda: pub.get_subscription_count()==0)
    process = launch(3)
    wait(lambda: has('interrupted_case', 'STOP_UNCONFIRMED'))
    blocked = copy.deepcopy(TASK)
    blocked['task_id'] = 'blocked_after_restart'
    send(blocked)
    wait(lambda: has('blocked_after_restart', 'REJECTED'))
    checks.append('Interrupted task restores unknown-stop lock and blocks new motion')
    report = {'passed':True, 'checks':checks, 'hardware_connected':False, 'motion_backend':'mock'}
    (OUT/'ros_report.json').write_text(json.dumps(report, indent=2)+'\n')
    (OUT/'ros_events.json').write_text(json.dumps(events, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False))
finally:
    if process:
        stop(process)
    for log in logs:
        log.close()
    node.destroy_node()
    rclpy.shutdown()
