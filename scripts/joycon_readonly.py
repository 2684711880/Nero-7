#!/usr/bin/env python3
"""Inspect Joy-Con joystick events; never connects to a robot."""
import glob
import os
import select
import struct
import sys

EVENT = struct.Struct('IhBB')


def main():
    devices = {}
    found = []
    for path in sorted(glob.glob('/dev/input/js*')):
        name_path = '/sys/class/input/' + os.path.basename(path) + '/device/name'
        try:
            with open(name_path, encoding='utf-8') as f:
                name = f.read().strip()
            if name in ('Nintendo Switch Left Joy-Con', 'Nintendo Switch Right Joy-Con',
                        'Nintendo Switch Combined Joy-Cons'):
                found.append((path, name))
        except OSError as exc:
            print(f'{path}: {exc}', file=sys.stderr)
    combined = [item for item in found if item[1] == 'Nintendo Switch Combined Joy-Cons']
    chosen = combined if combined else found
    if not combined and {name for _, name in chosen} != {
        'Nintendo Switch Left Joy-Con', 'Nintendo Switch Right Joy-Con'
    }:
        print('未找到组合 Joy-Con 或一对独立 Joy-Con。', file=sys.stderr)
        return 1
    for path, name in chosen:
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            devices[fd] = (path, name)
        except OSError as exc:
            print(f'{path}: {exc}', file=sys.stderr)
            for fd in devices:
                os.close(fd)
            return 1
    for path, name in devices.values():
        print(f'{name}: {path}', flush=True)
    if combined:
        print('当前是组合设备，左右手柄事件均从此设备输出。', flush=True)
    print('移动摇杆或按键观察事件；Ctrl+C 退出。不发送机械臂命令。', flush=True)
    try:
        while True:
            ready, _, _ = select.select(devices, [], [])
            for fd in ready:
                data = os.read(fd, EVENT.size * 32)
                if not data:
                    print(f'{devices[fd][1]} 已断开', file=sys.stderr)
                    return 1
                for offset in range(0, len(data) - EVENT.size + 1, EVENT.size):
                    _, value, kind, number = EVENT.unpack_from(data, offset)
                    initial = bool(kind & 0x80)
                    kind &= 0x7f
                    label = {1: 'button', 2: 'axis'}.get(kind, f'type{kind}')
                    print(f'{devices[fd][1]} {label}[{number}]={value}'
                          + (' (initial)' if initial else ''), flush=True)
    except KeyboardInterrupt:
        print('\n已退出。')
    finally:
        for fd in devices:
            os.close(fd)
    return 0


if __name__ == '__main__':
    sys.exit(main())
