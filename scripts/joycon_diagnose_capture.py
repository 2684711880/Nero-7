#!/usr/bin/env python3
"""Read-only timing capture of both Joy-Cons. No ROS, CAN writes or robot commands."""
import argparse
import csv
import json
import os
from pathlib import Path
import select
import time

from joycon_imu import JoyconIMU
from joycon_preview import EVENT, open_combined


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error('--seconds must be between 1 and 300')
    args.output.mkdir(parents=True, exist_ok=True)
    fd, path, axis_map = open_combined()
    imus = {}
    try:
        for side in ('left', 'right'):
            imus[side] = JoyconIMU(side)
        axes = {}
        counts = {side: 0 for side in imus}
        last_kernel = {side: None for side in imus}
        last_arrival = {side: None for side in imus}
        start = time.monotonic()
        next_tick = start
        next_msg = start
        with (args.output/'imu.csv').open('w', newline='') as imu_file, \
             (args.output/'loop.csv').open('w', newline='') as loop_file:
            iw, lw = csv.writer(imu_file), csv.writer(loop_file)
            iw.writerow(['host_s','side','kernel_us','kernel_gap_ms','batch_arrival_gap_ms'])
            lw.writerow(['host_s','loop_ms','read_left_ms','read_right_ms','stick_left_active','stick_right_active'])
            prev_tick = None
            while time.monotonic()-start < args.seconds:
                if select.select([fd], [], [], 0)[0]:
                    data = os.read(fd, EVENT.size*32)
                    if not data:
                        raise RuntimeError('组合摇杆设备断开')
                    for i in range(0,len(data)-EVENT.size+1,EVENT.size):
                        _, value, kind, index = EVENT.unpack_from(data, i)
                        if kind & 0x7f == 2: axes[index] = value
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(min(next_tick-now,.002))
                    continue
                next_tick += .02
                if now-next_tick > .02: next_tick = now+.02
                reads = {}
                for side, imu in imus.items():
                    before = time.monotonic()
                    samples = imu.read()
                    arrival = time.monotonic()
                    reads[side] = (arrival-before)*1000
                    batch_gap = None if last_arrival[side] is None else (arrival-last_arrival[side])*1000
                    for index, (stamp, _gyro) in enumerate(samples):
                        previous = last_kernel[side]
                        gap = None if previous is None else ((stamp-previous)&0xffffffff)/1000
                        iw.writerow([round(arrival-start,6),side,stamp,'' if gap is None else round(gap,3),
                                     round(batch_gap,3) if index == 0 and batch_gap is not None else ''])
                        last_kernel[side] = stamp
                        counts[side] += 1
                    if samples: last_arrival[side] = arrival
                active = {}
                for side, h, v in [('left',0,1),('right',3,4)]:
                    active[side] = any(abs(axes.get(axis_map[code],0)) > 6553 for code in (h,v))
                lw.writerow([round(now-start,6),'' if prev_tick is None else round((now-prev_tick)*1000,3),
                             round(reads['left'],3),round(reads['right'],3),int(active['left']),int(active['right'])])
                prev_tick = now
                if now >= next_msg:
                    print(f'已记录 {now-start:.0f}/{args.seconds:.0f} 秒；摇杆活动 left={active["left"]} right={active["right"]}',flush=True)
                    next_msg = now+5
        summary = {'duration_s': round(time.monotonic()-start,2),'samples':counts,'joystick':path,
                   'imu_devices':{s:d.path for s,d in imus.items()},'motion_commands_sent':False}
        (args.output/'capture.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
        print(json.dumps(summary,ensure_ascii=False),flush=True)
    finally:
        for imu in imus.values(): imu.close()
        os.close(fd)


if __name__ == '__main__': main()
