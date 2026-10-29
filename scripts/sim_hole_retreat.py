#!/usr/bin/env python3
"""Return the simulated right arm along its saved, validated outbound paths."""
import copy
import argparse
import json
import time
from pathlib import Path

import rclpy
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState, RobotTrajectory
from moveit_msgs.srv import GetStateValidity
from rosidl_runtime_py.set_message import set_message_fields

from sim_hole_execute import Executor
from sim_hole_observe import check_domain
from sim_hole_plan import Planner

ROOT = Path(__file__).resolve().parents[1] / 'sim_ws/validation/hole_approach/latest'


def reverse_trajectory(data):
    trajectory = copy.deepcopy(data)
    joint_trajectory = trajectory['joint_trajectory']
    points = joint_trajectory['points']
    if len(points) < 2:
        raise ValueError('Trajectory has fewer than two points')
    def nanos(point):
        t = point['time_from_start']
        return int(t['sec']) * 1_000_000_000 + int(t['nanosec'])
    duration = nanos(points[-1])
    if any(nanos(a) >= nanos(b) for a, b in zip(points, points[1:])):
        raise ValueError('Trajectory times are not strictly increasing')
    joint_trajectory['header']['stamp'] = {'sec': 0, 'nanosec': 0}
    for point in points:
        seconds, nanoseconds = divmod(duration - nanos(point), 1_000_000_000)
        point['time_from_start'] = {'sec': seconds, 'nanosec': nanoseconds}
        point['velocities'] = [-v for v in point['velocities']]
    joint_trajectory['points'] = list(reversed(points))
    return trajectory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='validate the path without moving')
    args = parser.parse_args()
    check_domain()
    plan = json.loads((ROOT / 'plan.json').read_text())
    execution = json.loads((ROOT / 'execution.json').read_text())
    observe = json.loads((ROOT / 'observe_plan.json').read_text())
    if (plan.get('status') != 'planned' or plan.get('source') != 'sim_depth'
            or execution.get('status') != 'executed_simulation'
            or observe.get('status') != 'executed_simulation'):
        raise RuntimeError('需要先完成本轮观察和视觉穿孔，才能沿原路回退')
    trajectories = [reverse_trajectory(plan[key]) for key in ('feed', 'approach')]
    trajectories.append(reverse_trajectory(observe['trajectory']))
    rclpy.init()
    executor, planner = Executor(), Planner()
    try:
        executor.validate_sim()
        deadline = time.monotonic() + 10
        while executor.joints is None and time.monotonic() < deadline:
            rclpy.spin_once(executor, timeout_sec=.1)
        if executor.joints is None:
            raise RuntimeError('没有收到仿真关节状态')
        state = RobotState()
        state.joint_state = executor.joints
        current = dict(zip(state.joint_state.name, state.joint_state.position))
        names = [f'left_joint{i}' for i in range(1, 8)]
        for saved in (plan['initial_joint_positions'], observe['initial_joint_positions']):
            if any(n not in current or n not in saved or abs(current[n] - saved[n]) > .02 for n in names):
                raise RuntimeError('左臂已经改变位置，不能直接重放旧的回退轨迹')
        first = trajectories[0]['joint_trajectory']
        if any(n not in current or abs(current[n] - q) > .03 for n, q in zip(first['joint_names'], first['points'][0]['positions'])):
            raise RuntimeError('右臂不在本轮穿孔终点，不能直接重放旧的回退轨迹')
        # Recheck the entire path against the current MoveIt scene before moving.
        for trajectory in trajectories:
            jt = trajectory['joint_trajectory']
            for point in jt['points']:
                positions = dict(current)
                positions.update(zip(jt['joint_names'], point['positions']))
                state.joint_state.position = [positions[name] for name in state.joint_state.name]
                state.joint_state.velocity = []
                state.joint_state.effort = []
                request = GetStateValidity.Request()
                request.robot_state = state
                request.group_name = ''
                if not planner.call(planner.valid, request).valid:
                    raise RuntimeError('当前场景中回退轨迹有碰撞，未执行')
        if args.check:
            print('回退路径检查通过，未发送动作。', flush=True)
            return
        if not executor.action.wait_for_server(timeout_sec=10):
            raise RuntimeError('仿真轨迹执行服务不可用')
        for label, trajectory in zip(('退出孔', '退回观察位', '返回观察前位置'), trajectories):
            msg = RobotTrajectory()
            set_message_fields(msg, trajectory)
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = msg
            handle = executor.wait(executor.action.send_goal_async(goal), 15)
            if not handle.accepted or executor.wait(handle.get_result_async(), 120).result.error_code.val != 1:
                raise RuntimeError(label + '失败，已停止后续回退')
            print(label + '完成（仿真）', flush=True)
        execution['retreated'] = True
        (ROOT / 'execution.json').write_text(json.dumps(execution, indent=2) + '\n')
    finally:
        planner.destroy_node()
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
