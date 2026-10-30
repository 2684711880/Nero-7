#!/usr/bin/env python3
"""Capture kernel gyro counts, event times, integrated orientation and mapping clamp; no CAN."""
import argparse
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
from joycon_imu import GYRO_RES, JoyconIMU
from joycon_move_p_dual import MAX_ROTATION


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=90)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    imus = {}
    metadata = dict(motion_commands_sent=False, rotation_limit_deg=math.degrees(MAX_ROTATION), arms={})
    try:
        for side in ('left', 'right'):
            imus[side] = JoyconIMU(side)
        calibration = {side: [] for side in imus}
        print('IMU静止校准3秒，请不要拿起手柄。', flush=True)
        end = time.monotonic() + 3
        while time.monotonic() < end:
            for side, imu in imus.items():
                calibration[side].extend(rate for _, rate, _ in imu.read_timed())
            time.sleep(.002)
        for side, imu in imus.items():
            imu.zero(calibration[side])
            metadata['arms'][side] = dict(device=imu.path, samples=0, bias_deg_s=np.rad2deg(imu.bias).tolist(),
                                         peak_gyro_deg_s=[0., 0., 0.], peak_angle_deg=0.,
                                         saturated_samples=0, max_event_gap_s=0., skipped_gaps=0)
        print('校准完成，可以开始分轴转动手柄；采集90秒，机械臂不会运动。', flush=True)
        (args.output / 'calibrated.json').write_text(json.dumps(metadata, indent=2))
        start = time.monotonic()
        with (args.output / 'gyro_mapping.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['elapsed_s', 'side', 'event_time_s', 'msc_timestamp_us',
                             'gyro_count_x', 'gyro_count_y', 'gyro_count_z',
                             'gyro_deg_s_x', 'gyro_deg_s_y', 'gyro_deg_s_z',
                             'relative_roll_deg', 'relative_pitch_deg', 'relative_yaw_deg',
                             'relative_total_angle_deg', 'mapped_total_angle_deg', 'saturated', 'event_gap_s'])
            while time.monotonic() - start < args.seconds:
                for side, imu in imus.items():
                    stats = metadata['arms'][side]
                    for stamp, rate, event_time in imu.read_timed():
                        gap = 0. if imu.last_event_time is None else event_time - imu.last_event_time
                        stats['max_event_gap_s'] = max(stats['max_event_gap_s'], gap)
                        if gap > .03:
                            stats['skipped_gaps'] += 1
                        imu.update(stamp, rate, event_time)
                        degrees_s = np.rad2deg(rate)
                        counts = np.rint(degrees_s * GYRO_RES).astype(int)
                        rpy = imu.orientation.as_euler('xyz', degrees=True)
                        angle = math.degrees(imu.orientation.magnitude())
                        limited = min(angle, math.degrees(MAX_ROTATION))
                        stats['samples'] += 1
                        stats['peak_gyro_deg_s'] = np.maximum(stats['peak_gyro_deg_s'], np.abs(degrees_s)).tolist()
                        stats['peak_angle_deg'] = max(stats['peak_angle_deg'], angle)
                        stats['saturated_samples'] += int(angle > math.degrees(MAX_ROTATION))
                        writer.writerow([round(time.monotonic()-start, 6), side, event_time, stamp,
                                         *counts.tolist(), *degrees_s.tolist(), *rpy.tolist(),
                                         angle, limited, int(angle > math.degrees(MAX_ROTATION)), gap])
                time.sleep(.002)
        metadata['duration_s'] = time.monotonic() - start
        (args.output / 'report.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(metadata, ensure_ascii=False, indent=2), flush=True)
    finally:
        for imu in imus.values():
            imu.close()


if __name__ == '__main__':
    main()
