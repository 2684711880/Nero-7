#!/usr/bin/env python3
"""Read live right-arm feedback and compare an approximate bottle tip to a saved hole; never sends commands."""
import argparse, json, socket, struct, select, time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
from handeye_charuco import PassiveJoints, FK
from inspect_orbbec_hole_target import mount, calibration, ROOT
from real_hole_geometry import transform_depth_observation


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('capture',type=Path,help='Directory saved with s in the 336L preview')
    ap.add_argument('--tool',type=Path,default=ROOT/'config/bottle_tool_preview.json')
    ap.add_argument('--extrinsic',type=Path,default=ROOT/'captures/orbbec_hole_observation_20261029_190542/observation.json')
    ap.add_argument('--output',type=Path,default=ROOT/'records/bottle_alignment_inspection.json')
    a=ap.parse_args(); path=a.capture/'metadata.json'; obs=json.loads(path.read_text()); ext=json.loads(a.extrinsic.read_text()); cfg=json.loads(a.tool.read_text())
    if not 0 <= time.time()-path.stat().st_mtime <= 30: raise ValueError('相机帧超过 30 秒，请按 s 重新采集')
    if obs['serial']!=ext['serial'] or not np.allclose(obs['depth_intrinsics'],ext['depth_intrinsics']):raise ValueError('相机或深度内参与外参记录不一致')
    d=obs.get('detection')
    if not d:raise ValueError('当前帧未检测到洞')
    axis=np.asarray(cfg['bottle_axis_from_grasp_to_bottom'],float);origin=np.asarray(cfg['grasp_center_from_flange_m'],float);length=float(cfg['grasp_to_bottom_m'])
    if axis.shape!=(3,) or origin.shape!=(3,) or not np.isfinite(axis).all() or not np.isfinite(origin).all() or not .01 <= length <= .3 or np.linalg.norm(axis)<1e-8:raise ValueError('工具参数无效')
    axis/=np.linalg.norm(axis); tool=origin+axis*length
    joints=PassiveJoints('can0')
    try:
        end=time.monotonic()+5
        while True:
            try:q,_=joints.snapshot(time.monotonic());break
            except ValueError:
                if time.monotonic()>end:raise
                time.sleep(.1)
    finally:joints.close()
    t=np.eye(4);t[:3,:3]=ext['depth_to_color_sdk_rotation'];t[:3,3]=np.asarray(ext['depth_to_color_sdk_translation_mm'])/1000
    center,normal=transform_depth_observation(mount('left'),calibration(ROOT/'archive/handeye/fixed_336l/obzg2.launch.py'),t,d['center_m'],d['normal'])
    flange=mount('right')@FK(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf')(q)
    tip=(flange@np.r_[tool,1])[:3];vec=center-tip
    report={'time':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds'),'status':'inspection_only','motion_sent':False,'camera_frame':str(path.resolve()),'camera_serial':obs['serial'],'right_joints_deg':np.rad2deg(q).tolist(),'assumed_tool_point_flange_m':tool.tolist(),'assumed_axis_flange':axis.tolist(),'estimated_tip_torso_m':tip.tolist(),'hole_center_torso_m':center.tolist(),'hole_normal_torso':normal.tolist(),'tip_to_hole_torso_m':vec.tolist(),'distance_m':float(np.linalg.norm(vec)),'hole_diameter_m':d['diameter_m'],'warnings':['Not an executable target','Gripper center and configured bottle axis are model assumptions; actual bottle geometry is not verified','Observation and arm feedback are not hardware synchronized','Uses earlier SDK extrinsic for same camera/depth intrinsics; current color profile not checked','Camera and robot frame accuracy require independent validation']}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(json.dumps(report,indent=2,ensure_ascii=False));print('已保存',a.output)
if __name__=='__main__':main()
