#!/usr/bin/env python3
"""Own the local mapping launch lifecycle without stopping unrelated Gazebo jobs."""
import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.parse import urlparse


def process_info(pid):
    try:
        proc = Path('/proc') / str(pid)
        fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
        env = dict(item.split(b'=', 1) for item in (proc/'environ').read_bytes().split(b'\0') if b'=' in item)
        return dict(pid=int(pid), ppid=int(fields[1]), start=fields[19], state=fields[0],
                    argv=[s.decode(errors='replace') for s in (proc/'cmdline').read_bytes().split(b'\0') if s],
                    cwd=(proc/'cwd').resolve(), env=env)
    except (OSError, ValueError, IndexError):
        return None


def snapshot():
    return {int(p.name): info for p in Path('/proc').iterdir() if p.name.isdigit()
            if (info := process_info(int(p.name))) is not None}


def matching_launch(info, root, domain, uri):
    if info['cwd'] != root/'sim_ws':
        return False
    if info['env'].get(b'ROS_DOMAIN_ID', b'0').decode() != domain:
        return False
    if info['env'].get(b'GAZEBO_MASTER_URI', b'http://127.0.0.1:11345').decode() != uri:
        return False
    args = info['argv']
    return any(Path(arg).name == 'ros2' and args[i+1:i+4] ==
               ['launch', 'nero_mid360_sim', 'store_superlio.launch.py']
               for i, arg in enumerate(args))


def family(pid, processes):
    found = {pid}
    while True:
        extra = {p for p, info in processes.items() if info['ppid'] in found}
        if extra <= found:
            return {p: processes[p] for p in found if p in processes}
        found |= extra


def alive(info):
    current = process_info(info['pid'])
    return current is not None and current['start'] == info['start'] and current['state'] != 'Z'


def send(info, sig):
    if alive(info):
        try:
            os.kill(info['pid'], sig)
        except ProcessLookupError:
            pass


def stop_launch(info):
    owned = family(info['pid'], snapshot())
    print(f"[NERO] 停止本工程的上一套建图仿真 (PID {info['pid']})…", flush=True)
    # Give ROS launch first chance to shut down all its children normally.
    send(info, signal.SIGINT)
    for sig, timeout in [(None, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        if sig is not None:
            for child in owned.values():
                send(child, sig)
        deadline = time.monotonic() + timeout
        while any(alive(child) for child in owned.values()) and time.monotonic() < deadline:
            time.sleep(.1)
        if not any(alive(child) for child in owned.values()):
            return
    raise RuntimeError('上一套仿真未退出，未启动新实例')


def port_busy(host, port):
    try:
        with socket.create_connection((host, port), timeout=.3):
            return True
    except ConnectionRefusedError:
        return False
    except OSError as error:
        raise RuntimeError(f'无法检查 Gazebo 地址 {host}:{port}: {error}') from error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--stop', action='store_true', help='Stop this project mapping session and exit')
    args, launch_args = parser.parse_known_args()
    root = args.project_root.resolve()
    uri = os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11489')
    domain = os.environ.get('ROS_DOMAIN_ID', '89')
    endpoint = urlparse(uri)
    host, port = endpoint.hostname or '127.0.0.1', endpoint.port or 11345
    if host not in ('127.0.0.1', 'localhost', '::1'):
        raise RuntimeError('此自动重启入口仅支持本机 Gazebo 地址')
    # Serialize stop/start across terminals using the same project/domain/master.
    key = hashlib.sha256(f'{root}:{domain}:{uri}'.encode()).hexdigest()[:20]
    lock_path = Path('/tmp')/f'nero-mapping-{os.getuid()}-{key}.lock'
    with lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        old = [info for info in snapshot().values() if matching_launch(info, root, domain, uri)]
        for info in old:
            stop_launch(info)
        if args.stop:
            print('[NERO] 本工程建图仿真已停止。', flush=True)
            return 0
        if port_busy(host, port):
            raise RuntimeError(f'Gazebo {uri} 被其他仿真或无法确认归属的残留进程占用；未关闭它')
        print(f'[NERO] ROS_DOMAIN_ID={domain}  GAZEBO_MASTER_URI={uri}', flush=True)
        command = ['ros2', 'launch', 'nero_mid360_sim', 'store_superlio.launch.py', *launch_args]
        print('[NERO] 启动建图仿真：' + ' '.join(launch_args or ['默认参数']), flush=True)
        child = subprocess.Popen(command, cwd=root/'sim_ws', start_new_session=True)
        child_info = None
        interrupted = False

        def interrupt(signum, _frame):
            nonlocal interrupted
            interrupted = True
            if child_info is not None:
                send(child_info, signal.SIGINT)

        previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            # Keep the lock until the launch is visible and has opened its master.
            # A second invocation then finds and replaces this exact launch family.
            deadline = time.monotonic() + 30
            while child.poll() is None and not interrupted:
                child_info = process_info(child.pid) or child_info
                if port_busy(host, port) or time.monotonic() >= deadline:
                    break
                time.sleep(.1)
            fcntl.flock(lock, fcntl.LOCK_UN)
            while child.poll() is None and not interrupted:
                time.sleep(.2)
            if interrupted and child_info is not None:
                stop_launch(child_info)
            return child.wait()
        finally:
            if child.poll() is None:
                info = process_info(child.pid)
                if info is not None:
                    stop_launch(info)
                child.wait()
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError) as error:
        print(f'[NERO] {error}', file=sys.stderr)
        sys.exit(1)
