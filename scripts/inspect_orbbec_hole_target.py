#!/usr/bin/env python3
"""Inspect fixed-camera coordinates from a saved observation; never commands motion."""
import argparse, ast, json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from real_hole_geometry import transform_depth_observation

ROOT=Path(__file__).resolve().parents[1]

def mount(side):
    root=ET.parse(ROOT/'third_party/open_nero/open_nero_description/urdf/dual_nero.xacro').getroot()
    o=root.find(f"joint[@name='{side}_arm_joint']/origin")
    t=np.eye(4);t[:3,3]=np.fromstring(o.get('xyz'),sep=' ');t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy'),sep=' ')).as_matrix();return t

def calibration(path):
    tree=ast.parse(path.read_text()); matches=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Call):
            for k in node.keywords:
                if k.arg=='arguments':
                    try: matches.append(ast.literal_eval(k.value))
                    except (ValueError,TypeError): pass
    if len(matches)!=1:raise ValueError('Cannot uniquely read saved static transform')
    args=matches[0]; opts=dict(zip(args[::2],args[1::2]))
    if opts.get('--frame-id')!='base_link' or opts.get('--child-frame-id')!='orbbec336l_color_optical':raise ValueError('Unexpected calibration frame names')
    t=np.eye(4);t[:3,3]=[float(opts['--'+x]) for x in ('x','y','z')];q=np.array([float(opts['--q'+x]) for x in ('x','y','z','w')]);
    if abs(np.linalg.norm(q)-1)>.001:raise ValueError('Invalid calibration quaternion')
    t[:3,:3]=Rotation.from_quat(q).as_matrix();return t

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('observation',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    data=json.loads(a.observation.read_text());d=data['detection']
    if d.get('status')!='detected':raise ValueError('Observation has no detected hole; no target calculated')
    if data.get('serial')!='CPCA963000AD':raise ValueError('Camera serial is not the recorded 336L; verify calibration provenance')
    color_depth=np.eye(4);color_depth[:3,:3]=data['depth_to_color_sdk_rotation'];color_depth[:3,3]=np.array(data['depth_to_color_sdk_translation_mm'])/1000.
    color=calibration(ROOT/'archive/handeye/fixed_336l/obzg2.launch.py')
    left=mount('left');right=mount('right')
    center,normal=transform_depth_observation(left,color,color_depth,d['center_depth_m'],d['normal_depth'])
    center_right=(np.linalg.inv(right)@np.r_[center,1])[:3];normal_right=right[:3,:3].T@normal
    out={'status':'inspection_only','motion_sent':False,'source_observation':str(a.observation.resolve()),'camera_serial':data['serial'],'calibration':'archive/handeye/fixed_336l/obzg2.launch.py','mount_model':'third_party/open_nero/open_nero_description/urdf/dual_nero.xacro','center_torso_m':center.tolist(),'normal_torso':normal.tolist(),'center_right_base_m':center_right.tolist(),'normal_right_base':normal_right.tolist(),'diameter_m':d['diameter_m'],'warning':'Model mount and calibration not independently validated; bottle TCP not measured; never use as an execution target'}
    print(json.dumps(out,indent=2,ensure_ascii=False))
    if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
if __name__=='__main__':main()
