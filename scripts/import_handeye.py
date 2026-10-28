#!/usr/bin/env python3
"""Import RViz static-transform output without executing its Python source."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from handeye_charuco import ROOT, transform, solve, write_json


def read_launch(path):
    tree = ast.parse(Path(path).read_text())
    lists = [ast.literal_eval(k.value) for n in ast.walk(tree) if isinstance(n, ast.Call)
             for k in n.keywords if k.arg == 'arguments']
    if len(lists) != 1:
        raise ValueError('Expected exactly one static transform arguments list')
    args = lists[0]
    values = dict(zip(args[::2], args[1::2]))
    if values.get('--frame-id') != 'gripper_flange' or values.get('--child-frame-id') != 'handeye_color_optical':
        raise ValueError('Expected gripper_flange -> handeye_color_optical')
    q = np.array([float(values['--q'+axis]) for axis in 'xyzw'])
    if abs(np.linalg.norm(q)-1) > 1e-3:
        raise ValueError('Invalid quaternion')
    q /= np.linalg.norm(q)
    x, y, z, w = q
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    t = transform(r, [float(values['--'+axis]) for axis in 'xyz'])
    if not np.isfinite(t).all():
        raise ValueError('Nonfinite calibration')
    return t


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose', type=Path, required=True)
    parser.add_argument('--samples', type=Path, required=True)
    parser.add_argument('--can', required=True)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    x = read_launch(args.pose)
    raw = yaml.safe_load(args.samples.read_text())
    samples = [dict(T_base_gripper=np.array(s['effector_wrt_world']).reshape(4,4).tolist(),
                    T_camera_board=np.array(s['object_wrt_sensor']).reshape(4,4).tolist()) for s in raw]
    result = solve(samples)
    park = np.array(result['T_gripper_camera'])
    result.update(T_gripper_camera=x.tolist(), T_camera_gripper=np.linalg.inv(x).tolist(),
                  status='pending_independent_pointing_validation', can=args.can, camera_serial=args.serial,
                  base_frame='base_link', gripper_frame='gripper_flange', camera_frame='handeye_color_optical',
                  source_pose=str(args.pose.resolve()), source_samples=str(args.samples.resolve()),
                  source_pose_sha256=hashlib.sha256(args.pose.read_bytes()).hexdigest(),
                  saved_vs_park_translation_mm=float(np.linalg.norm(x[:3,3]-park[:3,3])*1000),
                  saved_vs_park_rotation_deg=float(np.linalg.norm(cv2.Rodrigues(x[:3,:3].T@park[:3,:3])[0])*180/np.pi))
    # The solved sample statistics refer to PARK; saved output is independently retained.
    result['statistics_method'] = 'PARK recomputed from saved RViz samples'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, result)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
