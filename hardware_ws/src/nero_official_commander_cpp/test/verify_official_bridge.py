#!/usr/bin/env python3
"""No hardware: C++ -> isolated ROS subscriber -> real upstream callback with mock SDK."""
import json
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import PoseStamped, PoseArray
from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


def main():
    root = Path(__file__).resolve().parents[4]
    config = root / 'hardware_ws/src/nero_official_commander_cpp/config'
    rclpy.init()
    node = rclpy.create_node('official_bridge_verifier')
    received = {'line': [], 'arc': []}
    node.create_subscription(PoseStamped, '/official_verify/control/move_l', received['line'].append, 1)
    node.create_subscription(PoseArray, '/official_verify/control/move_c', received['arc'].append, 1)
    results = []

    def run(kind, send, extra=None, expected_success=True):
        command = ['ros2', 'run', 'nero_official_commander_cpp', 'commander_reference', '--ros-args',
                   '--params-file', str(config / f'{kind}.yaml'),
                   '-r', '__ns:=/official_verify', '-p', f'send:={str(send).lower()}']
        command += extra or []
        before = len(received[kind])
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic() + 15
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if process.poll() is None:
            process.kill()
            process.communicate()
            raise RuntimeError('C++ command timed out')
        output = process.communicate()[0]
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.02)
        if (process.returncode == 0) != expected_success:
            raise AssertionError(output)
        count = len(received[kind]) - before
        if count != (1 if send and expected_success else 0):
            raise AssertionError(f'Unexpected publish count: {count}\n{output}')
        results.append({'motion': kind, 'send': send, 'expected_success': expected_success,
                        'received_messages': count, 'passed': True})
        return received[kind][-1] if count else None

    try:
        run('line', False)
        run('arc', False)
        line = run('line', True)
        arc = run('arc', True)
        sdk = Mock()
        sdk.get_tcp2flange_pose.side_effect = lambda pose: pose
        stub = SimpleNamespace(agx_arm=sdk, _check_can_control=lambda: True)
        stub._create_pose_cmd = lambda pose: AgxArmRosNode._create_pose_cmd(stub, pose)
        AgxArmRosNode._move_l_callback(stub, line)
        sdk.move_l.assert_called_once_with([-0.4, 0.0, 0.3, 0.0, 0.0, 0.0])
        AgxArmRosNode._move_c_callback(stub, arc)
        sdk.move_c.assert_called_once_with(
            [-0.4, -0.2, 0.3, 0.0, 0.0, 0.0],
            [-0.4, 0.0, 0.4, 0.0, 0.0, 0.0],
            [-0.4, 0.2, 0.3, 0.0, 0.0, 0.0])
        results.append({'case': 'upstream_callbacks_to_mock_sdk', 'passed': True})
        run('line', True, ['-p', 'poses:=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]'], False)
        run('arc', True, ['-p', 'poses:=[0.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0,2.0,0.0,0.0,0.0,0.0,0.0,1.0]'], False)
        print(json.dumps(results, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
