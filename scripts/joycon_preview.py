#!/usr/bin/env python3
"""Read two Joy-Cons and ROS TCP feedback; print hypothetical targets only."""
import array
import fcntl
import glob
import os
import select
import struct
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation

EVENT = struct.Struct('IhBB')
AXIS_MAP_IOCTL = 0x80406A32
SPEED = 0.01  # metres per second at full stick deflection
DEADZONE = 0.10
MAX_OFFSET = 0.05


def mount_rotations():
    project = Path(__file__).resolve().parent.parent
    path = project / 'sim_ws/src/dual_nero_moveit_config/urdf/dual_nero.urdf'
    hardware_path = Path(get_package_share_directory('nero_dual_bringup')) / 'open_nero_description/urdf/dual_nero.xacro'
    root = ET.parse(path).getroot()
    hardware = ET.parse(hardware_path).getroot()
    result = {}
    for side in ('left', 'right'):
        origin = root.find(f"joint[@name='{side}_arm_joint']/origin")
        rpy = [float(value) for value in origin.attrib['rpy'].split()]
        result[side] = Rotation.from_euler('xyz', rpy)
        hw_origin = hardware.find(f"joint[@name='{side}_arm_joint']/origin")
        hw_rpy = [float(value) for value in hw_origin.attrib['rpy'].split()]
        if (result[side].inv() * Rotation.from_euler('xyz', hw_rpy)).magnitude() > 1e-6:
            raise RuntimeError(f'{side} 仿真与硬件模型的安装旋转不一致，停止预览')
        print(f'{side} 安装 RPY={rpy}（仿真及硬件模型一致）', flush=True)
    print(f'双臂安装模型: {path}', flush=True)
    return result


def open_combined():
    for path in sorted(glob.glob('/dev/input/js*')):
        with open('/sys/class/input/' + os.path.basename(path) + '/device/name') as f:
            if f.read().strip() != 'Nintendo Switch Combined Joy-Cons':
                continue
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        axes = array.array('B', [0] * 64)
        fcntl.ioctl(fd, AXIS_MAP_IOCTL, axes, True)
        indexes = {code: axes.index(code) for code in (0, 1, 3, 4)}
        return fd, path, indexes
    raise RuntimeError('未找到组合 Joy-Con，请确认两个手柄已连接并绑定')


def normalized(value):
    x = max(-1.0, min(1.0, value / 32767.0))
    return 0.0 if abs(x) < DEADZONE else x


class Preview(Node):
    def __init__(self, fd, axis_map, rotations):
        super().__init__('joycon_readonly_preview')
        self.fd, self.axis_map = fd, axis_map
        self.rotations = rotations
        self.axes = {}
        self.feedback = {}
        self.offset = {'left': [0.0, 0.0], 'right': [0.0, 0.0]}
        self.initial = {}
        self.last = time.monotonic()
        self.last_print = 0.0
        for side in ('left', 'right'):
            self.create_subscription(PoseStamped, f'/{side}/feedback/tcp_pose',
                                     lambda msg, side=side: self.on_pose(side, msg), 1)
        self.create_timer(0.05, self.update)

    def on_pose(self, side, msg):
        p = msg.pose.position
        self.feedback[side] = (time.monotonic(), (p.x, p.y, p.z))

    def update(self):
        while select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, EVENT.size * 32)
            if not data:
                raise RuntimeError('组合手柄已断开')
            for i in range(0, len(data) - EVENT.size + 1, EVENT.size):
                _, value, kind, number = EVENT.unpack_from(data, i)
                if kind & 0x7f == 2:
                    self.axes[number] = value
        now = time.monotonic()
        dt = min(now - self.last, 0.1)
        self.last = now
        if len(self.feedback) != 2 or any(now - v[0] > 0.5 for v in self.feedback.values()):
            if now - self.last_print >= 1:
                print('等待两臂新鲜 TCP 反馈；预览暂停。', flush=True)
                self.last_print = now
            return
        if not self.initial:
            self.initial = {side: self.feedback[side][1] for side in self.feedback}
        for side, hx, vy in (('left', 0, 1), ('right', 3, 4)):
            x = normalized(self.axes.get(self.axis_map[hx], 0))
            y = normalized(self.axes.get(self.axis_map[vy], 0))
            for i, value in enumerate((-y, x)):
                v = self.offset[side][i] + SPEED * value * dt
                self.offset[side][i] = max(-MAX_OFFSET, min(MAX_OFFSET, v))
        if now - self.last_print >= 0.5:
            for side in ('left', 'right'):
                p, d = self.initial[side], self.offset[side]
                # Robot common frame: +X forward, +Y left; stick right is -Y.
                base_delta = self.rotations[side].inv().apply([d[0], -d[1], 0.0])
                target = [p[i] + base_delta[i] for i in range(3)]
                print(f'{side}: 共同坐标偏移mm 前={d[0]*1000:+.1f} 右={d[1]*1000:+.1f} '
                      f'基座偏移mm=({base_delta[0]*1000:+.1f},{base_delta[1]*1000:+.1f},{base_delta[2]*1000:+.1f}) '
                      f'基座假设目标m=({target[0]:+.3f},{target[1]:+.3f},{target[2]:+.3f})', flush=True)
            self.last_print = now


def main():
    fd, path, axis_map = open_combined()
    print(f'组合手柄: {path}; 轴映射: {axis_map}', flush=True)
    print('仅打印假设目标，不发布控制命令；共同坐标系 +X 前、-Y 右，Ctrl+C 退出。', flush=True)
    print(f'预览速度 {SPEED} m/s，死区 {DEADZONE}，偏移限制 ±{MAX_OFFSET} m。', flush=True)
    rclpy.init()
    node = Preview(fd, axis_map, mount_rotations())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(fd)


if __name__ == '__main__':
    main()
