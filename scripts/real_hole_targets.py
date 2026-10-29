#!/usr/bin/env python3
"""Compute visual approach and feed targets; this command never moves the robot."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from handeye_charuco import FK
from pointing_model import mount_matrix
from real_hole_geometry import flange_goal

ROOT = Path(__file__).resolve().parents[1]

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--observation', type=Path, default=ROOT/'hardware_ws/validation/real_hole/observation.json')
    p.add_argument('--tip-mm', type=float, nargs=3, metavar=('X','Y','Z'), help='杆尖相对法兰，未填写时用仿真占位值仅作预览')
    p.add_argument('--axis', type=float, nargs=3, default=[0,0,1], metavar=('X','Y','Z'), help='杆的前向轴在右法兰坐标系中的方向，默认预览 +Z')
    p.add_argument('--diameter-mm', type=float, help='圆柱直径，mm')
    p.add_argument('--penetration-mm', type=float, default=30., help='前送穿过板面距离，默认预览 30 mm')
    p.add_argument('--standoff-mm', type=float, default=100.)
    p.add_argument('--output', type=Path, default=ROOT/'hardware_ws/validation/real_hole/targets.json')
    a = p.parse_args()
    supplied_tip = a.tip_mm is not None
    tip = a.tip_mm if supplied_tip else [0.,0.,180.]
    if not all(np.isfinite(v) for v in (*tip, *a.axis, a.penetration_mm, a.standoff_mm)) or (a.diameter_mm is not None and not np.isfinite(a.diameter_mm)):
        p.error('参数必须为有限数值')
    if (a.diameter_mm is not None and not 0 < a.diameter_mm) or not 0 < a.penetration_mm <= 100 or not 0 < a.standoff_mm <= 300:
        p.error('直径、前送距离及观察间距必须为正，前送不超过100 mm、间距不超过300 mm')
    obs = json.loads(a.observation.read_text())
    if obs.get('source') != 'right_d405_native_depth' or not 0 <= time.time()-obs.get('observed_wall_time', 0) <= 30:
        raise ValueError('需要 30 秒内的右 D405 孔观测；重新 capture')
    if a.diameter_mm is not None and a.diameter_mm >= obs['diameter_m'] * 1000:
        raise ValueError('杆直径不小于检测孔径')
    fk = FK(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf')
    current = mount_matrix('right') @ fk(np.asarray(obs['q_right_rad']))
    common = (obs['center_base_m'], obs['normal_base'], current[:3,:3], np.asarray(tip)/1000., a.axis)
    approach = flange_goal(*common, a.standoff_mm/1000.)
    feed = flange_goal(*common, -a.penetration_mm/1000.)
    report = dict(status='geometry_only', motion_sent=False, source_observation=str(a.observation),
                  observed_wall_time=obs['observed_wall_time'], q_left_rad=obs['q_left_rad'], q_right_rad=obs['q_right_rad'],
                  hole_center_base_m=obs['center_base_m'], outward_normal_base=obs['normal_base'],
                  tool_tip_flange_mm=tip, tool_axis_flange=a.axis, tool_diameter_mm=a.diameter_mm,
                  tool_geometry_supplied=bool(supplied_tip and a.diameter_mm is not None),
                  warning='杆尖/杆径缺失，使用仿真占位长度预览，不能作为杆尖实际位置' if not supplied_tip or a.diameter_mm is None else '需现场验证杆尖和碰撞体',
                  hole_diameter_mm=obs['diameter_m']*1000, standoff_mm=a.standoff_mm, penetration_mm=a.penetration_mm,
                  T_base_flange_approach=approach.tolist(), T_base_flange_feed=feed.tolist())
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2), flush=True)

if __name__ == '__main__':
    main()
