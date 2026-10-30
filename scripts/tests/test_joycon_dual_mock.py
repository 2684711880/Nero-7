#!/usr/bin/env python3
"""Isolated ROS mock integration: never connects to hardware or opens CAN."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Empty, SetBool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

assert os.environ.get('ROS_DOMAIN_ID') in ('103','104'), 'requires isolated ROS_DOMAIN_ID=103 or 104'
rclpy.init()
n = rclpy.create_node('fake_dual_driver')
stop_group = ReentrantCallbackGroup()
gates = {'left': [], 'right': []}
stops = {'left': 0, 'right': 0}
commands = {'left': [], 'right': []}
grippers = {'left': [], 'right': []}
pubs = {}
for side in ('left', 'right'):
    pubs[side] = (n.create_publisher(PoseStamped, f'/{side}/feedback/tcp_pose', 10),
                  n.create_publisher(JointState, f'/{side}/feedback/joint_states', 10))
    def gate(req, resp, side=side):
        gates[side].append(req.data); resp.success=True; return resp
    def stop(req, resp, side=side):
        stops[side] += 1
        if os.environ.get('MOCK_BLOCK') == '1' and side == 'left': time.sleep(.8)
        return resp
    def param(req, resp, side=side):
        resp.values = [ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=100),
                       ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='can1' if side=='left' else 'can0')]
        return resp
    n.create_service(SetBool, f'/{side}/control_enable', gate)
    n.create_service(Empty, f'/{side}/emergency_stop', stop, callback_group=stop_group)
    n.create_service(GetParameters, f'/{side}/agx_arm_ctrl_single_node/get_parameters', param)
    n.create_subscription(PoseStamped, f'/{side}/control/move_p', lambda msg, s=side: commands[s].append((time.monotonic(),msg)), 10)
    n.create_subscription(JointState, f'/{side}/control/joint_states', lambda msg, s=side: grippers[s].append(msg), 10)
def publish():
    for side, (pp, jp) in pubs.items():
        pose=PoseStamped();pose.header.stamp=n.get_clock().now().to_msg();pose.pose.orientation.w=1.;pp.publish(pose)
        msg=JointState();msg.header.stamp=n.get_clock().now().to_msg();msg.name=[f'joint{i}' for i in range(1,8)]+['gripper'];msg.position=[0.]*7+[.12 if os.environ.get('MOCK_GRIPPER_BINARY')=='1' else .04];msg.velocity=[0.]*8;jp.publish(msg)
n.create_timer(.02, publish)
executor = MultiThreadedExecutor(num_threads=3); executor.add_node(n)
thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
child_code = r'''
import os,struct,threading,time,sys
sys.path.insert(0,os.environ['NERO_JOYCON_SCRIPTS'])
import joycon_move_p_dual as m
import numpy as np
from scipy.spatial.transform import Rotation
class FakeIMU:
    def __init__(self, side):
        self.side=side;self.start=time.monotonic();self.bias=np.zeros(3);self.orientation=Rotation.identity();self.last_host=time.monotonic()
    def read(self):
        self.last_host=time.monotonic()
        if os.environ.get('MOCK_CHATTER')=='1' and self.side=='left' and self.last_host-self.start>8:
            degrees=.6 if int((self.last_host-self.start)*5)%2 else .4
            self.orientation=Rotation.from_euler('x',degrees,degrees=True)
        return [(int(self.last_host*1e6) & 0xffffffff,np.zeros(3))]
    def zero(self, samples): pass
    def read_timed(self):
        return [(stamp,rate,self.last_host) for stamp,rate in self.read()]
    def update(self, stamp, rate, event_time=None): pass
    def close(self): pass
m.JoyconIMU=FakeIMU
readfd,writefd=os.pipe()
m.open_combined=lambda:(readfd,'mock',{0:0,1:1,3:2,4:3})
m.live_devices=lambda:True
def ioctl(fd, request, arr, mutate):
    for i,code in enumerate([0x136,0x137,0x13d,0x13e,0x138,0x139,0x135,0x13c]): arr[i]=code
m.fcntl.ioctl=ioctl
def emit(kind,index,value): os.write(writefd,struct.pack('IhBB',0,value,kind,index))
def feed():
    time.sleep(8)
    if os.environ.get('MOCK_CHATTER')=='1': return
    if os.environ.get('MOCK_BLOCK')=='1':
        emit(2,1,-32767);emit(2,3,-32767)
        time.sleep(1);emit(2,1,0)
        time.sleep(2);emit(2,3,0)
        return
    emit(2,1,-32767)
    time.sleep(1)
    if os.environ.get('MOCK_DISCONNECT') == '1':
        os.close(writefd);return
    emit(2,1,0)
    time.sleep(1)
    emit(2,3,-32767)
    time.sleep(1)
    emit(2,3,0)
    time.sleep(.5)
    if os.environ.get('MOCK_GRIPPER_BINARY') == '1':
        for _ in range(3):
            emit(1,4,1);emit(1,4,1);time.sleep(.1);emit(1,4,0);time.sleep(.2)
        emit(1,5,1);time.sleep(.1);emit(1,5,0)
        return
    emit(1,4,1);time.sleep(.2);emit(1,4,0)
threading.Thread(target=feed,daemon=True).start()
sys.argv=['joycon_move_p_dual.py','--execute']
m.main()
'''
child_env = dict(os.environ, NERO_JOYCON_SCRIPTS=str(Path(__file__).resolve().parents[1]))
child = subprocess.Popen([sys.executable, '-u', '-c', child_code], env=child_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
try:
    deadline=time.monotonic()+20
    disconnect = os.environ.get('MOCK_DISCONNECT') == '1'
    chatter = os.environ.get('MOCK_CHATTER') == '1'
    block = os.environ.get('MOCK_BLOCK') == '1'
    binary = os.environ.get('MOCK_GRIPPER_BINARY') == '1'
    while time.monotonic()<deadline and ((not commands['left'] or (not chatter and (not stops['left'] or (not disconnect and (not commands['right'] or not stops['right'] or (not block and not grippers['left'])))))) or (binary and (len(grippers['left'])<3 or len(grippers['right'])<1))):
        if child.poll() is not None: break
        time.sleep(.1)
    if chatter:
        time.sleep(1)
        assert stops['left']==0, f'orientation caused repeated stop: {stops}'
    if not disconnect and child.poll() is None: child.send_signal(signal.SIGINT)
    output=child.communicate(timeout=8)[0]
    assert child.returncode == (1 if disconnect else 0), output
    assert commands['left'] and (bool(commands['right']) == (not disconnect and not chatter)), output
    assert stops['left']==1 and stops['right']==(0 if disconnect or chatter else 1), (stops,output)
    if block:
        stamps=[item[0] for item in commands['right']]
        max_gap=max(b-a for a,b in zip(stamps,stamps[1:]))
        print(f'右臂目标最大间隔 {max_gap:.3f}s（左侧停止服务延时 0.8s）')
        assert max_gap < .15, (max_gap,output)
    if not disconnect and not chatter and not block:
        if binary:
            assert [msg.position[0] for msg in grippers['left']] == [0., .1, 0.], (grippers,output)
            assert [msg.position[0] for msg in grippers['right']] == [0.], (grippers,output)
            print('Binary gripper PASS: stale 120mm feedback, held key, independent sides; targets left=0/100/0mm right=0mm')
        else:
            assert len(grippers['left'])==1 and not grippers['right'], (grippers,output)
            assert grippers['left'][0].name == ['gripper'] and abs(grippers['left'][0].position[0])<1e-6
        for side in ('left','right'):
            stamps=[item[0] for item in commands[side]]
            rate=(len(stamps)-1)/(stamps[-1]-stamps[0])
            assert rate >= 30, (side,rate,output)
            print(f'{side}: mock command rate {rate:.1f} Hz')
    assert gates['left']==[True,False] and gates['right']==[True,False], (gates,output)
    print(output)
    print(f'PASS: mock commands left={len(commands["left"])}, right={len(commands["right"])}, stop={stops}, gates={gates}; no CAN')
finally:
    if child.poll() is None: child.kill(); child.wait()
    executor.shutdown(timeout_sec=3)
    if rclpy.ok(): rclpy.shutdown()
    thread.join(timeout=3)
    n.destroy_node()
