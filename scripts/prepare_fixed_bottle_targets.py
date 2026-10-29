#!/usr/bin/env python3
"""Generate fixed-336L bottle alignment targets for RViz preview; no motion or actuator commands."""
import argparse,json,os,time
from pathlib import Path
import numpy as np
from handeye_charuco import PassiveJoints, FK
from inspect_orbbec_hole_target import mount,calibration,ROOT
from real_hole_geometry import flange_goal,transform_depth_observation

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('capture',type=Path,help='Directory captured with s in the Orbbec preview')
    ap.add_argument('--tool',type=Path,default=ROOT/'config/bottle_tool_preview.json')
    ap.add_argument('--extrinsic',type=Path,default=Path(os.environ.get('ORBBEC_HOLE_EXTRINSIC', ROOT/'captures/orbbec_hole_observation_20261029_190542/observation.json')))
    ap.add_argument('--standoff-mm',type=float,default=100.)
    ap.add_argument('--penetration-mm',type=float,default=30.)
    ap.add_argument('--output',type=Path,default=ROOT/'records/fixed_bottle_targets.json')
    a=ap.parse_args(); p=a.capture/'metadata.json'; obs=json.loads(p.read_text()); ext=json.loads(a.extrinsic.read_text()); tool=json.loads(a.tool.read_text()); capture_time=p.stat().st_mtime
    if not 0<=time.time()-capture_time<=30:raise ValueError('相机帧超过 30 秒，请重新按 s 采集')
    if obs['serial']!=ext['serial'] or not np.allclose(obs['depth_intrinsics'],ext['depth_intrinsics']):raise ValueError('相机或深度内参与外参记录不一致')
    d=obs.get('detection')
    if not d:raise ValueError('本帧未检测到洞，请重新采集')
    if not (0<a.standoff_mm<=300 and 0<a.penetration_mm<=100):raise ValueError('孔前距离或前送距离超出范围')
    axis=np.asarray(tool['bottle_axis_from_grasp_to_bottom'],float);grasp=np.asarray(tool['grasp_center_from_flange_m'],float);length=float(tool['grasp_to_bottom_m'])
    if axis.shape!=(3,) or grasp.shape!=(3,) or not np.isfinite(axis).all() or not np.isfinite(grasp).all() or np.linalg.norm(axis)<1e-8 or not .01<=length<=.3:raise ValueError('瓶子工具参数无效')
    axis/=np.linalg.norm(axis);tip=grasp+axis*length
    buses={'left':PassiveJoints('can1'),'right':PassiveJoints('can0')}
    try:
        end=time.monotonic()+5
        while True:
            try:q={s:bus.snapshot(time.monotonic())[0] for s,bus in buses.items()};break
            except ValueError:
                if time.monotonic()>end:raise
                time.sleep(.1)
    finally:
        for bus in buses.values():bus.close()
    t=np.eye(4);t[:3,:3]=ext['depth_to_color_sdk_rotation'];t[:3,3]=np.asarray(ext['depth_to_color_sdk_translation_mm'])/1000
    center,normal=transform_depth_observation(mount('left'),calibration(ROOT/'archive/handeye/fixed_336l/obzg2.launch.py'),t,d['center_m'],d['normal'])
    current=mount('right')@FK(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf')(q['right'])
    approach=flange_goal(center,normal,current[:3,:3],tip,axis,a.standoff_mm/1000)
    feed=flange_goal(center,normal,current[:3,:3],tip,axis,-a.penetration_mm/1000)
    out={'status':'geometry_only','source':'fixed_orbbec_preview','motion_sent':False,'observed_wall_time':capture_time,'source_observation':str(p.resolve()),'q_left_rad':q['left'].tolist(),'q_right_rad':q['right'].tolist(),'hole_center_base_m':center.tolist(),'outward_normal_base':normal.tolist(),'hole_diameter_mm':d['diameter_m']*1000,'tool_tip_flange_mm':(tip*1000).tolist(),'tool_axis_flange':axis.tolist(),'tool_geometry_supplied':False,'standoff_mm':a.standoff_mm,'penetration_mm':a.penetration_mm,'T_base_flange_approach':approach.tolist(),'T_base_flange_feed':feed.tolist(),'warning':'仅供预览：孔板为估计模型，瓶子方向与夹持点为模型假设，瓶子及腕部相机碰撞体未完整接入；禁止直接执行实物动作。'}
    a.output.parent.mkdir(parents=True,exist_ok=True);tmp=a.output.with_suffix('.tmp');tmp.write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n');os.replace(tmp,a.output);print(json.dumps(out,indent=2,ensure_ascii=False));print('已保存预览目标:',a.output,flush=True)
if __name__=='__main__':main()
