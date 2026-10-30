#!/usr/bin/env python3
"""Own the teleop and driver process groups so Ctrl+C cannot orphan CAN writers."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import time


def group_alive(group):
    # Zombies cannot own CAN writers; their parent is responsible for reaping.
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == group and fields[0] != 'Z':
                return True
        except (OSError, ValueError, IndexError):
            continue
    return False


def stop_group(process, grace=3.):
    for sig, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
        if not group_alive(process.pid):
            break
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        end = time.monotonic() + timeout
        while time.monotonic() < end and group_alive(process.pid):
            process.poll()
            time.sleep(.05)
    process.wait(timeout=2)
    if group_alive(process.pid):
        raise RuntimeError(f'进程组 {process.pid} 未退出，禁止开始其他机械臂控制')


def main():
    root = Path(__file__).resolve().parents[1]
    driver = teleop = None
    # Ignore repeated Ctrl+C only during cleanup, after first notifying teleop.
    try:
        driver = subprocess.Popen(['ros2', 'launch', str(root / 'scripts/joycon_drivers.launch.py'),
                                   'speed_percent:=100'], start_new_session=True)
        teleop = subprocess.Popen([sys.executable, '-u', str(root / 'scripts/joycon_move_p_dual.py'),
                                   '--execute', '--speed-percent', '100'], start_new_session=True)
        return teleop.wait()
    except KeyboardInterrupt:
        return 130
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if teleop is not None:
                stop_group(teleop, grace=8.)
        finally:
            if driver is not None:
                stop_group(driver)
        print('遥操作及本次官方驱动进程组已退出。', flush=True)


if __name__ == '__main__':
    sys.exit(main())
