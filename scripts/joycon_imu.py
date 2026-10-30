"""Read kernel Joy-Con IMU events; orientation is relative and gyro-only.

The kernel maps both hands to X=triggers, Y=left, Z=out of buttons.
Gyro is 14247 units per degree/s. No magnetometer/yaw correction exists.
"""
import fcntl
import glob
import math
import os
import struct
import time
import numpy as np
from scipy.spatial.transform import Rotation

INPUT_EVENT = struct.Struct('llHHi')
GYRO_RES = 14247.0
IMU_TIMEOUT = 2.0
MAX_INTEGRATION_GAP = .03


class JoyconIMU:
    def __init__(self, side):
        wanted = f'Nintendo Switch {side.title()} Joy-Con IMU'
        self.fd = None
        for name_path in glob.glob('/sys/class/input/event*/device/name'):
            try:
                with open(name_path) as f:
                    name = f.read().strip()
                if name == wanted:
                    device = '/dev/input/' + name_path.split('/')[-3]
                    self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
                    self.path = device
                    break
            except OSError:
                continue
        if self.fd is None:
            raise RuntimeError(f'无法读取 {wanted}，检查设备连接和输入设备权限')
        # EVIOCSCLOCKID(CLOCK_MONOTONIC): host event time is independent of wall-clock jumps.
        fcntl.ioctl(self.fd, 0x400445a0, struct.pack('i', time.CLOCK_MONOTONIC))
        self.values = np.zeros(3)
        for i, code in enumerate((3, 4, 5)):
            buf = bytearray(24)
            fcntl.ioctl(self.fd, 0x80184540 + code, buf, True)
            self.values[i] = struct.unpack('iiiiii', buf)[0]
        self.sample_timestamp = None
        self.last_timestamp = None
        self.last_event_time = None
        self.sample_dt = .005
        self.last_host = None
        self.bias = np.zeros(3)
        self.orientation = Rotation.identity()

    def read_timed(self):
        samples = []
        while True:
            try:
                data = os.read(self.fd, INPUT_EVENT.size * 128)
            except BlockingIOError:
                break
            if not data:
                raise RuntimeError(f'{self.path} disconnected')
            for i in range(0, len(data)-INPUT_EVENT.size+1, INPUT_EVENT.size):
                sec, usec, kind, code, value = INPUT_EVENT.unpack_from(data, i)
                if kind == 3 and 3 <= code <= 5:
                    self.values[code-3] = value
                elif kind == 4 and code == 5:
                    self.sample_timestamp = value & 0xffffffff
                elif kind == 0 and code == 0 and self.sample_timestamp is not None:
                    timestamp = self.sample_timestamp
                    self.last_host = time.monotonic()
                    samples.append((timestamp, np.deg2rad(self.values / GYRO_RES), sec + usec * 1e-6))
        return samples

    def read(self):
        # Preserve raw MSC timestamps for diagnostics.
        return [(stamp, rate) for stamp, rate, _ in self.read_timed()]

    def update(self, timestamp, angular_rate, event_time=None):
        if event_time is not None:
            if self.last_event_time is not None:
                elapsed = event_time - self.last_event_time
                if elapsed < 0:
                    raise RuntimeError(f'{self.path} IMU 单调时钟倒退')
                if elapsed > IMU_TIMEOUT:
                    raise RuntimeError(f'{self.path} IMU 事件间隔 {elapsed:.3f}s，超过 {IMU_TIMEOUT:.1f}s 阈值')
                if elapsed > MAX_INTEGRATION_GAP:
                    print(f'{self.path}: IMU 事件间隔 {elapsed:.3f}s，跳过缺失时段的姿态积分', flush=True)
                else:
                    # Three samples normally arrive in each Bluetooth report.
                    # Distinguish report boundaries from samples in one burst.
                    if elapsed > .003:
                        self.sample_dt = min(max(elapsed / 3, .001), .01)
                    rate = np.asarray(angular_rate) - self.bias
                    rate[np.abs(rate) < math.radians(1)] = 0
                    self.orientation = self.orientation * Rotation.from_rotvec(rate * self.sample_dt)
            self.last_event_time = event_time
            return self.orientation
        if self.last_timestamp is not None:
            dt = ((timestamp - self.last_timestamp) & 0xffffffff) * 1e-6
            if dt > IMU_TIMEOUT:
                raise RuntimeError(f'{self.path} IMU 样本间隔 {dt:.3f}s，超过 {IMU_TIMEOUT:.1f}s 阈值')
            if MAX_INTEGRATION_GAP < dt <= IMU_TIMEOUT:
                # Never extrapolate the missing angular motion. Orientation will
                # be approximate after packet loss until the next calibration.
                print(f'{self.path}: IMU 间隔 {dt:.3f}s，跳过缺失时段的姿态积分', flush=True)
            if 0 < dt <= MAX_INTEGRATION_GAP:
                rate = np.asarray(angular_rate) - self.bias
                # Reduce stationary drift; angular rate below 1 degree/s is ignored.
                rate[np.abs(rate) < math.radians(1)] = 0
                self.orientation = self.orientation * Rotation.from_rotvec(rate * dt)
        self.last_timestamp = timestamp
        return self.orientation

    def zero(self, gyro_samples):
        data = np.asarray(gyro_samples, dtype=float)
        if len(data) < 100 or data.ndim != 2 or data.shape[1] != 3:
            raise RuntimeError('IMU 静止校准样本不足')
        if not np.isfinite(data).all() or np.max(np.std(data, axis=0)) > math.radians(2):
            raise RuntimeError('IMU 校准期间手柄发生运动，请静止后重试')
        self.bias = np.mean(data, axis=0)
        self.orientation = Rotation.identity()
        self.last_timestamp = None
        self.last_event_time = None
        self.sample_dt = .005

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
