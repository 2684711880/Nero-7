#!/usr/bin/env python3
"""Single NERO 1.21: official leader mode, without joint/pose targets."""
import argparse
import select
import socket
import struct
import time

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config


def check_feedback(channel):
    """Require fresh raw feedback for every joint before any enable command."""
    frames = {}
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.bind((channel,))
        end = time.monotonic() + 2
        while time.monotonic() < end:
            if not select.select([bus], [], [], 0.1)[0]:
                continue
            cid, size, data = struct.unpack('=IB3x8s', bus.recv(16))
            if cid in (0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A9) and size == 8:
                frames[cid] = (time.monotonic(), data)
            if len(frames) == 5 and all(time.monotonic()-t < 0.1 for t, _ in frames.values()):
                break
    if len(frames) != 5 or any(time.monotonic()-t > 0.1 for t, _ in frames.values()):
        raise RuntimeError('七轴或状态反馈缺失/过期，未发送使能。')
    status = frames[0x2A1][1]
    if status[1] not in (0, 6) or int.from_bytes(status[6:8], 'big'):
        raise RuntimeError(f'状态异常 {status.hex()}，未发送使能。')
    angles = []
    for cid in (0x2A5, 0x2A6, 0x2A7, 0x2A9):
        values = struct.unpack('>ii', frames[cid][1])
        angles.extend(v / 1000 for v in (values[:1] if cid == 0x2A9 else values))
    print('当前七轴角度(度):', angles, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can', required=True, choices=['can0', 'can1'])
    parser.add_argument('--execute', action='store_true', help='使能并切换官方零力拖动模式')
    args = parser.parse_args()
    if args.execute:
        parser.error('执行入口已封禁：实机出现非预期摆动，原因未查明。禁止使能或切换模式。')
    check_feedback(args.can)
    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=args.can))
    attempted_enable = False
    try:
        robot.connect()
        firmware = robot.get_firmware()
        print('固件:', firmware, flush=True)
        if not firmware or firmware['software_version'] != '1.21':
            raise RuntimeError('此脚本只核对了当前 1.21 固件，停止。')
        if not args.execute:
            print('只读检查完成；加 --execute 才会使能/切模式。')
            return
        check_feedback(args.can)
        print('即将释放抱闸，请托住臂；不发送目标角度或回零命令。', flush=True)
        attempted_enable = True
        if not robot.enable(timeout=2.0):
            raise RuntimeError('使能未确认，停止进入拖动。')
        robot.set_leader_mode()
        print('已发送零力拖动模式指令，实际效果需现场观察。Ctrl+C 退出拖动。', flush=True)
        while True:
            status = robot.get_arm_status()
            print('状态:', status.msg if status else '暂无状态', flush=True)
            time.sleep(1)
    except KeyboardInterrupt:
        print('\n退出拖动。', flush=True)
    finally:
        try:
            if attempted_enable:
                robot.set_follower_mode()
                print('已发送 follower 退出指令；未自动失能。请托住臂确认退出，勿将断开程序当作急停。', flush=True)
                time.sleep(0.2)
        finally:
            robot.disconnect()


if __name__ == '__main__':
    main()
