#!/usr/bin/env python3
"""Plan a user-selected right-arm observation posture; explicit do executes."""
import argparse
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
import threading
from real_hole_capture import Feedback
from real_hole_preview import reset_preview, preview_publisher


def wait(future, timeout):
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if not event.wait(timeout) or future.result() is None:
        raise RuntimeError('ROS request timed out')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    targets = parser.add_mutually_exclusive_group(required=True)
    targets.add_argument('--joints-deg', nargs=7, type=float, metavar=('J1','J2','J3','J4','J5','J6','J7'))
    targets.add_argument('--wrist-joint', '--joint', dest='wrist_joint', type=int, choices=(2,4,5,6,7), help='相对当前位置调整的右臂关节')
    parser.add_argument('--delta-deg', type=float, default=5., help='右腕关节相对增量，度；默认 +5')
    parser.add_argument('--plan-only', action='store_true', help='仅预览，不等待执行输入')
    parser.add_argument('--left-gripper-mm', type=float, help='左夹爪开口 mm；默认读左臂反馈')
    args = parser.parse_args()
    if (args.joints_deg is not None and not np.isfinite(args.joints_deg).all()) or (args.left_gripper_mm is not None and (not np.isfinite(args.left_gripper_mm) or not 0 <= args.left_gripper_mm <= 100)):
        parser.error('Joint targets must be finite')
    if args.wrist_joint is not None and (not np.isfinite(args.delta_deg) or not 0 < abs(args.delta_deg) <= 20):
        parser.error('--delta-deg 必须非零且在 ±20° 以内')
    rclpy.init()
    node = Node('nero_real_hole_observe')
    feedback = Feedback()
    merged = {}
    def receive_merged(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        for name, value in zip(message.name, message.position):
            merged[name] = (value, stamp)
    merged_sub = node.create_subscription(JointState, '/joint_states', receive_merged, 1)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(feedback)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    pub = None
    try:
        deadline = time.monotonic()+10
        while True:
            try:
                measured = feedback.stationary()
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.1)
        state = RobotState()
        grip = merged.get('right_gripper')
        if grip is None or not 0 <= time.time() - grip[1] < .3:
            raise RuntimeError('Missing fresh right gripper feedback for collision planning')
        for side in ('left', 'right'):
            names = [f'{side}_joint{i}' for i in range(1, 8)]
            state.joint_state.name.extend(names)
            state.joint_state.position.extend(measured[side].tolist())
        state.joint_state.name.extend(['left_gripper', 'right_gripper'])
        left_width = feedback.gripper_width('left') if args.left_gripper_mm is None else args.left_gripper_mm / 1000.
        state.joint_state.position.extend([left_width, grip[0]])
        state.is_diff = False
        client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('MoveIt planning service unavailable')
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = 'right_arm'
        motion.pipeline_id = 'ompl'
        motion.start_state = state
        motion.allowed_planning_time = 10.
        motion.num_planning_attempts = 5
        motion.max_velocity_scaling_factor = .1
        motion.max_acceleration_scaling_factor = .1
        wanted = np.deg2rad(args.joints_deg) if args.joints_deg is not None else measured['right'].copy()
        if args.wrist_joint is not None:
            wanted[args.wrist_joint-1] += np.deg2rad(args.delta_deg)
            print(f"右腕 J{args.wrist_joint}: {np.rad2deg(measured['right'][args.wrist_joint-1]):.2f}° → {np.rad2deg(wanted[args.wrist_joint-1]):.2f}°，先预览。", flush=True)
            # Keep the remaining right-arm joints near their measured positions
            # throughout the path, rather than only constraining the final state.
            for i, value in enumerate(measured['right'], 1):
                if i != args.wrist_joint:
                    motion.path_constraints.joint_constraints.append(JointConstraint(joint_name=f'right_joint{i}', position=float(value), tolerance_above=.005, tolerance_below=.005, weight=1.))
        constraint = Constraints()
        for i, q in enumerate(wanted, 1):
            tolerance = .001 if args.wrist_joint is not None else .005
            constraint.joint_constraints.append(JointConstraint(joint_name=f'right_joint{i}', position=float(q), tolerance_above=tolerance, tolerance_below=tolerance, weight=1.))
        motion.goal_constraints = [constraint]
        response = wait(client.call_async(request), 45).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(f'Observation planning failed: {response.error_code.val}')
        if args.wrist_joint is not None:
            trajectory = response.trajectory.joint_trajectory
            expected_names = {f'right_joint{i}' for i in range(1,8)}
            if set(trajectory.joint_names) != expected_names or not trajectory.points:
                raise RuntimeError('右腕预览轨迹的关节集合不正确')
            largest = 0.
            for point in trajectory.points:
                values = dict(zip(trajectory.joint_names, point.positions))
                largest = max(largest, max(abs(values[f'right_joint{i}']-measured['right'][i-1]) for i in range(1,8) if i != args.wrist_joint))
            if largest > .0051:
                raise RuntimeError('规划中其他右臂关节偏离当前位置，拒绝执行')
            print(f'其余右臂关节在整条轨迹中最大变化 {np.rad2deg(largest):.3f}°。', flush=True)
        pub = preview_publisher(node)
        display = DisplayTrajectory()
        display.model_id = 'dual_nero'
        display.trajectory_start = response.trajectory_start
        display.trajectory = [response.trajectory]
        pub.publish(display)
        if args.plan_only:
            print('轨迹已发布 RViz，仅预览，没有发送运动。', flush=True)
            time.sleep(1.)
            return
        print('观察位轨迹已发布到 RViz。确认实物空间和轨迹后，输入 do 执行；其他输入取消。', flush=True)
        try:
            confirmation = input('> ').strip()
        except EOFError:
            confirmation = ''
        if confirmation != 'do':
            print('已取消，没有发送运动。')
            return
        now = feedback.stationary()
        if any(np.max(np.abs(now[side] - measured[side])) > .02 for side in now):
            raise RuntimeError('规划后双臂位置变化，请重新规划')
        action = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        if not action.wait_for_server(timeout_sec=5):
            raise RuntimeError('MoveIt execution service unavailable')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.trajectory
        handle = wait(action.send_goal_async(goal), 15)
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected trajectory')
        result = wait(handle.get_result_async(), 120).result
        if result.error_code.val != 1:
            raise RuntimeError(f'Execution failed: {result.error_code.val}')
        print('控制器报告观察轨迹完成，请查看实物反馈。', flush=True)
    finally:
        if pub is not None and not args.plan_only:
            try: reset_preview(node, pub)
            except Exception as error: print(f'预览复位未完成：{error}', flush=True)
        executor.shutdown()
        thread.join()
        feedback.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
