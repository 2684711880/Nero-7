#!/usr/bin/env python3
"""Stop this project's Joy-Con control processes; never enables or homes arms."""
import os
from pathlib import Path
import signal
import time

ROOT = Path(__file__).resolve().parents[1]


def find_processes():
    matches = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / 'cmdline').read_bytes().decode().strip('\0').split('\0')
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                continue
            role = None
            if str(ROOT / 'scripts/joycon_process.py') in args:
                role = 'supervisor'
            elif str(ROOT / 'scripts/joycon_move_p_dual.py') in args and '--execute' in args:
                role = 'teleop'
            elif (str(ROOT / 'hardware_ws/install/agx_arm_ctrl/lib/agx_arm_ctrl/agx_arm_ctrl_single') in args
                  and any(arg in args for arg in ('__ns:=/left', '__ns:=/right'))):
                role = 'driver'
            elif str(ROOT / 'scripts/joycon_drivers.launch.py') in args and 'launch' in args:
                role = 'launcher'
            if role:
                matches[int(entry.name)] = (role, fields[19])
        except (OSError, UnicodeError, ValueError, IndexError):
            continue
    return matches


def signal_and_wait(roles, sig, timeout):
    processes = {pid: value for pid, value in find_processes().items() if value[0] in roles}
    for pid, identity in processes.items():
        if find_processes().get(pid) != identity:
            continue
        try:
            os.kill(pid, sig)
            print(f'请求 {identity[0]} PID={pid} 退出（{sig.name}）。', flush=True)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current = find_processes()
        if not any(current.get(pid) == identity for pid, identity in processes.items()):
            return
        time.sleep(.1)


def stop_all():
    if not find_processes():
        print('当前没有本工程的Joy-Con遥操作或官方双臂驱动进程。', flush=True)
        return
    # Give the normal teleop cleanup time to request hold and close both gates.
    signal_and_wait({'supervisor'}, signal.SIGINT, 20.)
    signal_and_wait({'teleop'}, signal.SIGINT, 8.)
    # A teleop stuck after its cleanup deadline is ended before driver teardown.
    signal_and_wait({'teleop'}, signal.SIGTERM, 2.)
    signal_and_wait({'teleop'}, signal.SIGKILL, 2.)
    for sig, timeout in ((signal.SIGINT, 3.), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
        signal_and_wait({'driver', 'launcher', 'supervisor'}, sig, timeout)
    remaining = find_processes()
    if remaining:
        raise RuntimeError(f'仍有控制进程未退出：{remaining}；不要启动归位')
    print('本工程遥操作和官方双臂驱动已退出，可以手动执行 --return。未自动归位。', flush=True)


if __name__ == '__main__':
    stop_all()
