#!/usr/bin/env python3
"""Disable with the official SDK only after actual joint feedback is near park."""
import argparse
import math
import sys
import time
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
from move_to_park import PARK, CHANNELS

TOLERANCE_DEG = 3.0
MAX_AGE = 0.5


class FeedbackPending(RuntimeError):
    pass


def fresh(message, label, since=0.0):
    if (message is None or not since <= message.timestamp <= time.time()
            or time.time()-message.timestamp > MAX_AGE):
        raise FeedbackPending(f'{label}反馈缺失或过期')
    return message.msg


def angles(robot):
    # 固定 SDK 的聚合 timestamp 仅对应最后一组，必须分别核对七轴的四组帧。
    for group in ('joint_12', 'joint_34', 'joint_56', 'joint_7'):
        fresh(getattr(robot._parser, group, None), group)
    values = list(fresh(robot.get_joint_angles(), '关节角度'))
    if len(values) != 7 or not all(math.isfinite(q) for q in values):
        raise RuntimeError('关节角度必须是七轴有限数值；拒绝失能。')
    return values


def enable_flags(robot, since=0.0):
    # SDK 的 get_joints_enable_status_list 会把缺失反馈当 False，不能用它证明已失能。
    return [bool(fresh(robot.get_driver_states(i), f'J{i}使能状态', since).foc_status.driver_enable_status)
            for i in range(1, 8)]


def check_park(robot, side):
    q = angles(robot)
    enable_flags(robot)
    errors = [abs(math.degrees(actual-target)) for actual, target in zip(q, PARK)]
    print(f'[{side}] 实际角度(度)：{[round(math.degrees(v), 2) for v in q]}', flush=True)
    outside = [f'J{i+1}偏差{error:.2f}°' for i, error in enumerate(errors) if error > TOLERANCE_DEG]
    if outside:
        raise RuntimeError(f"[{side}] 不在 park ±{TOLERANCE_DEG:g}° 内：{', '.join(outside)}；拒绝失能。")


def main():
    parser = argparse.ArgumentParser(description='默认检查双臂 park±3° 后失能；独立官方 SDK，无需 ROS/RViz。')
    parser.add_argument('arm', nargs='?', default='both', choices=['both', 'left', 'right'])
    args = parser.parse_args()
    sides = ('left', 'right') if args.arm == 'both' else (args.arm,)
    robots = {}
    try:
        for side in sides:
            robot = AgxArmFactory.create_arm(create_agx_arm_config(
                robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=CHANNELS[side]))
            robots[side] = robot
            robot.connect()
        # 所选机械臂全部通过检查之后才发送任何失能指令。
        for side, robot in robots.items():
            deadline = time.monotonic()+5
            while True:
                try:
                    check_park(robot, side)
                    break
                except FeedbackPending:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.01)
        # 在实际发送之前再次核对所有角度，避免初始化等待期间姿态改变。
        for side, robot in robots.items():
            check_park(robot, side)
        requests = {}
        for side, robot in robots.items():
            requests[side] = time.time()
            robot.disable(timeout=0.0)
            print(f'[{side}/{CHANNELS[side]}] 已发送失能。', flush=True)
        pending = set(robots)
        deadline = time.monotonic()+5
        while pending:
            for side in tuple(pending):
                try:
                    if not any(enable_flags(robots[side], requests[side])):
                        print(f'[{side}] 新反馈确认七轴均已失能。', flush=True)
                        pending.remove(side)
                except FeedbackPending:
                    pass
            if pending and time.monotonic() >= deadline:
                raise RuntimeError(f"失能反馈未确认：{', '.join(sorted(pending))}；请检查现场状态。")
            time.sleep(0.02)
        return 0
    except KeyboardInterrupt:
        print('\n已中断，请确认现场使能状态。', file=sys.stderr)
        return 130
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        for robot in robots.values():
            robot.disconnect()


if __name__ == '__main__':
    sys.exit(main())
