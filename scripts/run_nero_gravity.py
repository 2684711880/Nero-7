#!/usr/bin/env python3
"""Start one or two copies of the official configured example; no robot-control logic."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

WORKER = Path(__file__).with_name('nero_gc_official_configured.py')


def run_both():
    children = []
    old_term = signal.getsignal(signal.SIGTERM)
    old_int = signal.getsignal(signal.SIGINT)

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    code = 0
    try:
        for side in ('left', 'right'):
            # Separate sessions: Ctrl+C reaches the launcher, which forwards it once.
            children.append(subprocess.Popen(
                [sys.executable, str(WORKER), '--side', side], start_new_session=True))
        while True:
            for child in children:
                result = child.poll()
                if result is not None:
                    code = result if result >= 0 else 128 - result
                    return code
            time.sleep(0.1)
    except KeyboardInterrupt:
        return 130
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            for child in children:
                if child.poll() is None:
                    try:
                        child.send_signal(signal.SIGINT)
                    except ProcessLookupError:
                        pass
            for child in children:
                child.wait()
        finally:
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGTERM, old_term)


def main():
    parser = argparse.ArgumentParser(description='默认启动双臂官方重力补偿；left=can1，right=can0。')
    parser.add_argument('arms', nargs='?', default='both', choices=['both', 'left', 'right'])
    parser.add_argument('--arm', choices=['both', 'left', 'right'], help='也可用 --arm left/right/both 选择')
    parser.add_argument('--execute', action='store_true', help='兼容旧命令；不加此参数也会直接启动')
    args = parser.parse_args()
    selected = args.arm or args.arms
    if selected == 'both':
        return run_both()
    os.execv(sys.executable, [sys.executable, str(WORKER), '--side', selected])


if __name__ == '__main__':
    sys.exit(main())
