#!/usr/bin/env python3
"""Experimental dual Joy-Con Cartesian teleop via official move_p ROS driver.

No MoveIt collision checking. Requires running official drivers, speed_percent=100,
and arms already enabled. No automatic enable, homing or disable.
"""
import argparse
import array
import copy
import fcntl
import glob
import math
import os
import select
import time
import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from joycon_preview import EVENT, mount_rotations, normalized, open_combined
from joycon_imu import IMU_TIMEOUT, JoyconIMU

PERIOD = .02
LOOKAHEAD = .002
MAX_SPEED = 1.25
MAX_ROTATION = math.radians(90)
MAX_ROTATION_STEP = math.radians(1)
ORIENTATION_TOLERANCE = math.radians(.5)
GRIPPER_OPEN_WIDTH = .1  # Official wrapper's open() default, metres.
GRIPPER_CLOSE_WIDTH = 0.
BUTTON_MAP_IOCTL = 0x81906A34


def live_devices():
    names = set()
    for path in glob.glob('/sys/class/input/js*/device/name'):
        try:
            with open(path) as f:
                names.add(f.read().strip())
        except OSError:
            continue
    return {'Nintendo Switch Left Joy-Con', 'Nintendo Switch Right Joy-Con',
            'Nintendo Switch Combined Joy-Cons'}.issubset(names)


def target_from_stick(pose, horizontal, vertical, rotation, up=False, down=False):
    h, v = normalized(horizontal), normalized(vertical)
    z = float(up) - float(down)
    length = math.sqrt(h*h + v*v + z*z)
    if length < .2:
        return None
    scale = LOOKAHEAD / max(1.0, length)
    delta = rotation.inv().apply([-v * scale, -h * scale, z * scale])
    out = copy.deepcopy(pose)
    for key, value in zip(('x', 'y', 'z'), delta):
        setattr(out.pose.position, key, getattr(out.pose.position, key) + float(value))
    return out


def orientation_step(pose, anchor, handset_rotation, mount):
    relative = handset_rotation.as_rotvec()
    angle = np.linalg.norm(relative)
    if angle > MAX_ROTATION:
        relative = relative * (MAX_ROTATION / angle)
    rel_world = type(mount).from_rotvec(relative)
    q0, q = anchor.pose.orientation, pose.pose.orientation
    initial = type(mount).from_quat([q0.x, q0.y, q0.z, q0.w])
    current = type(mount).from_quat([q.x, q.y, q.z, q.w])
    desired = mount.inv() * rel_world * mount * initial
    error = (current.inv() * desired).as_rotvec()
    error_norm = np.linalg.norm(error)
    if error_norm <= ORIENTATION_TOLERANCE:
        return None
    step = error * min(MAX_ROTATION_STEP/error_norm, 1.0)
    return (current * type(mount).from_rotvec(step)).as_quat()


class Teleop(Node):
    def __init__(self, rotations):
        super().__init__('joycon_move_p_dual')
        self.rotations = rotations
        self.pose = {}
        self.joints = {}
        self.moving = {'left': False, 'right': False}
        self.translating = {'left': False, 'right': False}
        self.stop_pending = {'left': None, 'right': None}
        self.ready = {'left': False, 'right': False}
        # Commanded state only; never infer toggle direction from measured width.
        # Unknown on startup: the first explicit button press requests close.
        self.gripper_open_target = {'left': None, 'right': None}
        self.pub, self.gate, self.stop, self.params = {}, {}, {}, {}
        self.gripper = {}
        for side in ('left', 'right'):
            self.create_subscription(PoseStamped, f'/{side}/feedback/tcp_pose',
                                     lambda msg, s=side: self.pose.update({s: msg}), 10)
            self.create_subscription(JointState, f'/{side}/feedback/joint_states',
                                     lambda msg, s=side: self.joints.update({s: msg}), 10)
            self.pub[side] = self.create_publisher(PoseStamped, f'/{side}/control/move_p', 1)
            self.gripper[side] = self.create_publisher(JointState, f'/{side}/control/joint_states', 1)
            self.gate[side] = self.create_client(SetBool, f'/{side}/control_enable')
            self.stop[side] = self.create_client(Empty, f'/{side}/emergency_stop')
            self.params[side] = self.create_client(GetParameters, f'/{side}/agx_arm_ctrl_single_node/get_parameters')

    def fresh(self, side):
        pose, joints = self.pose.get(side), self.joints.get(side)
        if pose is None or joints is None:
            raise RuntimeError(f'{side}: feedback missing')
        now = self.get_clock().now().nanoseconds * 1e-9
        for msg in (pose, joints):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if not 0 <= now - stamp < .3:
                raise RuntimeError(f'{side}: stale feedback')
        if list(joints.name[:7]) != [f'joint{i}' for i in range(1, 8)] or len(joints.position) < 7 or len(joints.velocity) < 7:
            raise RuntimeError(f'{side}: incomplete joint feedback')
        q = pose.pose.orientation
        p = pose.pose.position
        vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w] + list(joints.position[:7]) + list(joints.velocity[:7])
        if not all(math.isfinite(v) for v in vals) or abs(sum(v*v for v in [q.x,q.y,q.z,q.w])-1)>.02:
            raise RuntimeError(f'{side}: invalid feedback')
        if max(abs(v) for v in joints.velocity[:7]) > MAX_SPEED:
            raise RuntimeError(f'{side}: measured speed exceeded {MAX_SPEED} rad/s')
        return pose

    def call(self, client, request, timeout=3):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service unavailable: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'service timeout: {client.srv_name}')
        return future.result()

    def halt(self, side):
        if self.moving[side]:
            if self.stop_pending[side] is not None:
                return
            if not self.stop[side].service_is_ready():
                raise RuntimeError(f'{side}: official stop service unavailable')
            self.stop_pending[side] = self.stop[side].call_async(Empty.Request())
            self.moving[side] = False
            print(f'{side}: 已请求官方 emergency_stop（返回不证明实物停止）。', flush=True)

    def poll_stops(self):
        for side, future in self.stop_pending.items():
            if future is not None and future.done():
                if future.exception() is not None:
                    raise RuntimeError(f'{side}: stop request failed: {future.exception()}')
                self.stop_pending[side] = None

    def toggle_gripper(self, side, execute):
        previous = self.gripper_open_target[side]
        target_open = False if previous is None else not previous
        width = GRIPPER_OPEN_WIDTH if target_open else GRIPPER_CLOSE_WIDTH
        command = JointState(name=['gripper'], position=[width], effort=[1.])
        if execute:
            self.gripper[side].publish(command)
        self.gripper_open_target[side] = target_open
        action = '完全张开' if target_open else '完全闭合'
        print(f'{side}: 夹爪目标 {action} {width:.3f}m' + ('' if execute else '（预览）'), flush=True)
        return command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='允许发送实际运动目标')
    parser.add_argument('--speed-percent', type=int, choices=range(1, 101), default=100, metavar='1..100',
                        help='要求官方驱动匹配的速度比例，默认100')
    args = parser.parse_args()
    fd, path, axis_map = open_combined()
    button_codes = array.array('H', [0] * 200)
    fcntl.ioctl(fd, BUTTON_MAP_IOCTL, button_codes, True)
    button_map = {'left': {'up': 0x136, 'down': 0x13d, 'grip': 0x138, 'reset': 0x135},
                  'right': {'up': 0x137, 'down': 0x13e, 'grip': 0x139, 'reset': 0x13c}}
    buttons = {side: {name: button_codes.index(code) for name, code in mapping.items()}
               for side, mapping in button_map.items()}
    rotations = mount_rotations()
    imus = {}
    try:
        imus = {side: JoyconIMU(side) for side in ('left', 'right')}
    except Exception:
        for imu in imus.values(): imu.close()
        os.close(fd)
        raise
    # Preserve Python's KeyboardInterrupt so cleanup can request stop and close gates.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Teleop(rotations)
    axes, pressed = {}, {}
    grip_pending = {'left': False, 'right': False}
    anchors = {}
    preview_last = {'left': 0.0, 'right': 0.0}
    gates_open = []
    try:
        print(f'{path} 键位={buttons}；速度百分比={args.speed_percent}；目标前瞻={LOOKAHEAD}m；目标发送频率={1/PERIOD}Hz', flush=True)
        print(f'相对初始姿态旋转上限={math.degrees(MAX_ROTATION):.0f}°；每帧最大旋转={math.degrees(MAX_ROTATION_STEP):.1f}°', flush=True)
        print('摇杆前后左右；L/R 上升；摇杆按压下降；ZL/ZR 夹爪切换；Capture/Home 回启动位置。无保持键。Ctrl+C 退出。', flush=True)
        print('ZL/ZR 每次按下交替完全闭合/张开；本次启动首次按下为完全闭合。', flush=True)
        print(f'无碰撞规划；复位只还原位置，不还原姿态；驱动速度{args.speed_percent}%；实测关节速度保护{MAX_SPEED} rad/s。', flush=True)
        print('IMU 校准：两手柄平放、按键朝上，肩键朝机器人前方，静止 2 秒。', flush=True)
        calibration = {side: [] for side in imus}
        calibration_end = time.monotonic() + 2.5
        while time.monotonic() < calibration_end:
            for side, imu in imus.items():
                calibration[side].extend(rate for _, rate in imu.read())
            time.sleep(.005)
        for side, imu in imus.items():
            imu.zero(calibration[side])
            print(f'{side}: IMU 校准 {len(calibration[side])} 样本，偏置 {np.rad2deg(imu.bias).round(2).tolist()} °/s', flush=True)
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            try:
                for side in ('left','right'): node.fresh(side)
                break
            except RuntimeError: pass
        else: raise RuntimeError('双臂反馈未就绪')
        for side, can in (('left', 'can1'), ('right', 'can0')):
            response = node.call(node.params[side], GetParameters.Request(names=['speed_percent','can_port']))
            if response.values[0].integer_value != args.speed_percent or response.values[1].string_value != can:
                raise RuntimeError(f'{side}: driver settings mismatch')
            if node.pub[side].get_subscription_count() != 1:
                raise RuntimeError(f'{side}: unexpected move_p subscribers')
        if args.execute:
            for side in ('left', 'right'):
                result = node.call(node.gate[side], SetBool.Request(data=True))
                if not result.success: raise RuntimeError(f'{side}: control gate rejected')
                gates_open.append(side)
            print('控制门开启；先松开按键并将摇杆回中，再操作。', flush=True)
        else:
            print('预览模式：不打开控制门、不发送控制或停止命令。', flush=True)
        next_tick = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.005)
            node.poll_stops()
            for imu in imus.values():
                for stamp, gyro, event_time in imu.read_timed(): imu.update(stamp, gyro, event_time)
            while select.select([fd], [], [], 0)[0]:
                data = os.read(fd, EVENT.size * 32)
                if not data: raise RuntimeError('组合手柄断开')
                for i in range(0, len(data)-EVENT.size+1, EVENT.size):
                    _, value, kind, index = EVENT.unpack_from(data, i)
                    if kind & 0x7f == 2: axes[index] = value
                    elif kind & 0x7f == 1:
                        was_pressed = bool(pressed.get(index, 0))
                        pressed[index] = value
                        if not (kind & 0x80) and value and not was_pressed:
                            for side in ('left', 'right'):
                                if index == buttons[side]['grip'] and node.ready[side]:
                                    grip_pending[side] = True
            now = time.monotonic()
            if now < next_tick: continue
            next_tick += PERIOD
            if now-next_tick > PERIOD:
                next_tick = now+PERIOD
            if not live_devices(): raise RuntimeError('左右手柄或组合设备断开')
            for side, hcode, vcode in [('left',0,1), ('right',3,4)]:
                pose = node.fresh(side)
                if imus[side].last_host is None or now - imus[side].last_host > IMU_TIMEOUT:
                    raise RuntimeError(f'{side}: IMU 反馈中断')
                h, v = axes.get(axis_map[hcode],0), axes.get(axis_map[vcode],0)
                keys = buttons[side]
                up, down = bool(pressed.get(keys['up'], 0)), bool(pressed.get(keys['down'], 0))
                reset = bool(pressed.get(keys['reset'], 0))
                moving = target_from_stick(pose,h,v,rotations[side],up,down)
                translation_active = moving is not None
                neutral = (moving is None and not up and not down and not reset
                           and not pressed.get(keys['grip'], 0))
                if not node.ready[side] and neutral:
                    node.ready[side] = True
                    anchors[side] = copy.deepcopy(pose)
                    print(f'{side}: 输入已回中，记录当前复位位置。',flush=True)
                if node.ready[side] and reset:
                    p, origin = pose.pose.position, anchors[side].pose.position
                    delta = [origin.x-p.x, origin.y-p.y, origin.z-p.z]
                    distance = math.sqrt(sum(x*x for x in delta))
                    moving = copy.deepcopy(pose) if distance > .001 else None
                    if moving is not None:
                        for key, value in zip(('x','y','z'), delta):
                            setattr(moving.pose.position, key, getattr(p, key)+value*min(LOOKAHEAD/distance,1.0))
                    translation_active = moving is not None
                if node.ready[side]:
                    # Relative handset axes are aligned to the common robot frame at calibration.
                    quat = orientation_step(pose, anchors[side], imus[side].orientation, rotations[side])
                    if quat is not None:
                        if moving is None: moving = copy.deepcopy(pose)
                        moving.pose.orientation.x, moving.pose.orientation.y, moving.pose.orientation.z, moving.pose.orientation.w = [float(x) for x in quat]
                if grip_pending[side]:
                    grip_pending[side] = False
                    node.toggle_gripper(side, args.execute)
                active = node.ready[side] and moving is not None
                if active and node.stop_pending[side] is None:
                    if args.execute:
                        moving.header.stamp = node.get_clock().now().to_msg()
                        node.pub[side].publish(moving)
                        node.moving[side] = True
                    elif now-preview_last[side] >= .25:
                        quaternion = moving.pose.orientation
                        rpy = type(rotations[side]).from_quat([quaternion.x,quaternion.y,quaternion.z,quaternion.w]).as_euler('xyz', degrees=True)
                        hand = imus[side].orientation.as_euler('xyz', degrees=True)
                        print(f'{side}: 目标位置 {moving.pose.position}; 目标基座 RPY°={rpy.round(2).tolist()}; 手柄相对 RPY°={hand.round(2).tolist()}',flush=True)
                        preview_last[side] = now
                elif args.execute and node.translating[side]:
                    # Stop after translation input ends. Orientation settling alone
                    # must not repeatedly switch the arm from move_p to move_j.
                    node.halt(side)
                node.translating[side] = node.ready[side] and translation_active
    except KeyboardInterrupt:
        print('用户退出。',flush=True)
    finally:
        if args.execute:
            for side in ('left','right'):
                try: node.halt(side)
                except Exception as error: print(f'{side}: 停止请求失败 {error}',flush=True)
            for side in ('left','right'):
                future = node.stop_pending[side]
                if future is not None:
                    try:
                        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                        if not future.done() or future.exception() is not None:
                            print(f'{side}: 停止请求未确认', flush=True)
                    except Exception as error: print(f'{side}: 停止请求等待失败 {error}',flush=True)
            for side in gates_open:
                try:
                    result = node.call(node.gate[side], SetBool.Request(data=False))
                    print(f'{side}: 关闭控制门 {result.success}',flush=True)
                except Exception as error: print(f'{side}: 关闭控制门失败 {error}',flush=True)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        os.close(fd)
        for imu in imus.values(): imu.close()


if __name__ == '__main__': main()
