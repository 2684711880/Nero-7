#!/usr/bin/env python3
"""User-owned pointing launch; preview by default, execution only through RViz menu."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import signal
import threading
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
from handeye_charuco import ROOT, PassiveJoints, camera_config


def model_gripper_center_mm():
    robot=ET.parse(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf').getroot()
    base=robot.find("joint[@name='gripper_base_joint']/origin")
    fingers=[robot.find(f"joint[@name='gripper_joint{i}']/origin") for i in (1,2)]
    return ((np.fromstring(base.get('xyz'),sep=' ')+np.mean([np.fromstring(f.get('xyz'),sep=' ') for f in fingers],axis=0))*1000).tolist()


def pointing_environment(base):
    env=dict(base,ROS_DOMAIN_ID='77',ROS_LOCALHOST_ONLY='1')
    if env.get('RMW_IMPLEMENTATION')=='rmw_cyclonedds_cpp':
        env['CYCLONEDDS_URI']=(ROOT/'config/cyclonedds_pointing.xml').as_uri()
    return env


def stop_launch(child, grace=10):
    """Reap the launch's private process group, including orphaned SDK workers."""
    for sig, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        try: os.killpg(child.pid, sig)
        except ProcessLookupError: break
        try: child.wait(timeout=timeout)
        except subprocess.TimeoutExpired: continue
        # ros2 launch may have exited while its SDK child is still shutting down.
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            try: os.killpg(child.pid, 0)
            except ProcessLookupError: return
            time.sleep(.05)
    child.wait()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--can',required=True)
    p.add_argument('--other-can',default=None,help='Other arm feedback interface')
    p.add_argument('--tcp-offset-mm',type=float,nargs=3,default=None,help='Default: centre between model finger origins')
    p.add_argument('--standoff',type=float,default=.10)
    p.add_argument('--mock',action='store_true');p.add_argument('--no-rviz',action='store_true')
    a=p.parse_args()
    side={'can1':'left','can0':'right'}.get(a.can)
    if side is None:p.error('Use can1 for left or can0 for right')
    if a.other_can is None:a.other_can='can0' if side=='left' else 'can1'
    if a.can==a.other_can:p.error('The two arms require different CAN interfaces')
    if a.tcp_offset_mm is None:a.tcp_offset_mm=model_gripper_center_mm()
    if not np.isfinite(a.tcp_offset_mm).all() or not .03<=a.standoff<=.5:p.error('Invalid tip offset/standoff')
    if not a.mock:
        calibration=json.loads((ROOT/f'config/handeye/{side}/d405.json').read_text())
        if calibration['can']!=a.can:p.error('Calibration belongs to a different CAN interface')
        try:camera_config(calibration['camera_serial'])
        except (ValueError,RuntimeError) as exc:
            raise SystemExit(f'启动前检查失败：{exc}。请接好 D405 USB 后重试；尚未启动 ROS 控制栈。')
    lock=Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))/f'nero-handeye-{os.getuid()}.lock'
    with lock.open('w') as f:
        try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('Close the existing hand-eye camera window first; it owns the same camera.')
        initial=[0,1.2,0,.3,0,0,0]
        if not a.mock:
            bus=PassiveJoints(a.can)
            try:
                time.sleep(.8);initial=bus.snapshot(time.monotonic())[0].tolist()
            finally:bus.close()
        session=ROOT/'calibration_data'/('rviz_point_'+time.strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}')
        session.mkdir(parents=True)
        print(f'夹爪模型中心相对法兰(mm): {a.tcp_offset_mm}; 板前距离 {a.standoff}m\n日志: {session}',flush=True)
        env=pointing_environment(os.environ)
        # ROS-installed Python executables use /usr/bin/python3. Make numerical packages coherent.
        pure=subprocess.check_output([str(ROOT/'.venv-official/bin/python'),'-c','import sysconfig; print(sysconfig.get_path("purelib"))'],text=True).strip()
        env['PYTHONPATH']=str(ROOT/'scripts')+os.pathsep+pure+os.pathsep+env.get('PYTHONPATH','')
        command=['ros2','launch',str(ROOT/'scripts/pointing.launch.py'),f'can:={a.can}',
                 'tcp_offset_mm:='+json.dumps(a.tcp_offset_mm),'initial_q:='+json.dumps(initial),
                 f'standoff:={a.standoff}',f'side:={side}',f'other_can:={a.other_can}',f'session:={session}',f'mock:={str(a.mock).lower()}',
                 f'rviz:={str(not a.no_rviz).lower()}']
        child=subprocess.Popen(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                               text=True,bufsize=1,start_new_session=True)
        def log_output():
            with (session/'console.log').open('w') as log:
                for line in child.stdout:
                    log.write(line);log.flush()
                    try:sys.stdout.write(line);sys.stdout.flush()
                    except BrokenPipeError:pass
        logger=threading.Thread(target=log_output,daemon=True);logger.start()
        def terminate(signum,frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM,terminate)
        try:return child.wait()
        except KeyboardInterrupt:
            return 130
        finally:
            stop_launch(child)
            logger.join(timeout=2)


if __name__=='__main__':raise SystemExit(main())
