#!/usr/bin/env python3
"""Official AgileX gripper CAN frame; does not command arm joints."""
import argparse
import math
import select
import socket
import struct
import time

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('side', choices=('left', 'right'))
    parser.add_argument('action', choices=('open', 'close'))
    parser.add_argument('--force', type=float, required=True, help='目标力 N，范围 0.5–3.0')
    parser.add_argument('--width-mm', type=float, default=70., help='打开宽度 mm')
    parser.add_argument('--can', help='指定 CAN；默认 left=can1, right=can0')
    args = parser.parse_args()
    if not math.isfinite(args.force) or not .5 <= args.force <= 3:
        parser.error('--force 必须在 0.5–3.0 N 范围内')
    if not math.isfinite(args.width_mm) or not 0 < args.width_mm <= 100:
        parser.error('--width-mm 必须大于 0 且不超过 100 mm')
    channel = args.can or {'left': 'can1', 'right': 'can0'}[args.side]
    width_um = round(args.width_mm * 1000) if args.action == 'open' else 0
    payload = struct.pack('>ihBB', width_um, round(args.force * 1000), 1, 0)
    can = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        can.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack('=II', 0x2A8, 0x7FF))
        can.bind((channel,))
        before = None
        if select.select([can], [], [], .2)[0]:
            _, size, data = struct.unpack('=IB3x8s', can.recv(16))
            if size == 8:
                before = struct.unpack('>ihBB', data)
        can.send(struct.pack('=IB3x8s', 0x159, 8, payload))
        print(f'已通过 {channel} 发送 {args.side} 夹爪 {args.action}：目标 {width_um/1000:g} mm，力 {args.force:g} N。', flush=True)
        deadline = time.monotonic() + 2
        last = None
        while time.monotonic() < deadline:
            if select.select([can], [], [], max(0, deadline-time.monotonic()))[0]:
                _, size, data = struct.unpack('=IB3x8s', can.recv(16))
                if size == 8:
                    last = struct.unpack('>ihBB', data)
        if last is None:
            print('没有收到夹爪 0x2A8 反馈，无法确认动作。')
        else:
            value, force, status, mode = last
            unit = 'mm' if mode == 0 else '角度模式原始值'
            reading = value / 1000. if mode == 0 else value
            print(f'夹爪反馈：位置 {reading:g} {unit}，反馈力 {force/1000:g} N，状态 0x{status:02X}，使能={(status >> 6) & 1}。')
            if before is not None and mode == 0 and before[3] == 0:
                change_mm = (value - before[0]) / 1000.
                print(f'发送后开口变化：{change_mm:+.3f} mm。')
                if abs(change_mm) < .5 and abs(value-width_um) > 2000:
                    print('开口基本没有变化且未到目标，不能确认动作成功。')
            print('夹持物体时未到零开口可能是接触物体；反馈不等于夹持成功。')
    finally:
        can.close()

if __name__ == '__main__':
    main()
