#!/usr/bin/env python3
"""Standalone park using official SDK move_j; no ROS, MoveIt or RViz."""
import argparse
import math
import sys
import time
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

PARK = [math.radians(value) for value in (0, 85, 0, 5, 0, 0, 0)]
CHANNELS = {'left': 'can1', 'right': 'can0'}


def main():
    parser = argparse.ArgumentParser(description='官方 SDK 独立回 park；默认双臂，不需要 MoveIt/RViz。')
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
            robot.set_speed_percent(10)
            deadline = time.monotonic() + 5
            while not robot.enable():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'{side} 使能未确认。')
                time.sleep(0.1)
        print('官方 SDK move_j → park：J2=85°，J4=5°，其余=0°，速度比例10%。', flush=True)
        sent_at = time.time()
        for side, robot in robots.items():
            robot.move_j(PARK.copy())
            print(f'[{side}/{CHANNELS[side]}] 已发送 park。', flush=True)
        pending = set(robots)
        deadline = time.monotonic() + 30
        while pending:
            for side in tuple(pending):
                feedback = robots[side].get_joint_angles()
                if feedback is None or not sent_at <= feedback.timestamp <= time.time():
                    continue
                angles = feedback.msg
                if len(angles) == 7 and all(math.isfinite(q) and abs(q-target) <= math.radians(0.5)
                                            for q, target in zip(angles, PARK)):
                    print(f'[{side}] 反馈确认已到 park（各轴误差≤0.5°）。', flush=True)
                    pending.remove(side)
            if pending and time.monotonic() >= deadline:
                raise RuntimeError(f"等待 park 到达超时：{', '.join(sorted(pending))}；没有确认到位。")
            time.sleep(0.05)
        return 0
    except KeyboardInterrupt:
        for side, robot in robots.items():
            try:
                feedback = robot.get_joint_angles()
                if feedback is not None:
                    robot.move_j(list(feedback.msg))
            except Exception as error:
                print(f'[{side}] 请求当前位姿保持失败：{error}', file=sys.stderr)
        print('\n已请求保持当前反馈姿态。', flush=True)
        return 130
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        for robot in robots.values():
            robot.disconnect()


if __name__ == '__main__':
    sys.exit(main())
