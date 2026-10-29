#!/usr/bin/env python3
"""Capture one fixed-camera RGB-D hole observation; no arm or CAN access."""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
from pyorbbecsdk import Context, Pipeline
from depth_hole import detect_depth_hole
from preview_orbbec_hole import color_bgr, depth_geometry


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--serial')
    p.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'captures')
    p.add_argument('--min-diameter-mm', type=float, default=42.5)
    p.add_argument('--max-diameter-mm', type=float, default=120.)
    args = p.parse_args()
    if not 0 < args.min_diameter_mm < args.max_diameter_mm:
        p.error('Invalid diameter range')
    ctx = Context(); devices = ctx.query_devices()
    if devices.get_count() == 0:
        raise RuntimeError('没有发现奥比中光相机')
    if devices.get_count() != 1 and not args.serial:
        raise RuntimeError('存在多台相机，请指定 --serial')
    dev = devices.get_device_by_serial_number(args.serial) if args.serial else devices.get_device_by_index(0)
    if dev is None or '336' not in dev.get_device_info().get_name():
        raise RuntimeError('没有找到指定的 Gemini 336L')
    info = dev.get_device_info(); pipe = Pipeline(dev); started = False
    try:
        pipe.start(); started = True; frames = None
        for _ in range(30):
            item = pipe.wait_for_frames(1000)
            if item and item.get_depth_frame() and item.get_color_frame(): frames = item
        if frames is None: raise RuntimeError('未收到完整 RGB-D 帧')
        d, c = frames.get_depth_frame(), frames.get_color_frame()
        dp, cp = d.get_stream_profile(), c.get_stream_profile()
        k, maps, size = depth_geometry(dp.as_video_stream_profile())
        raw = np.frombuffer(d.get_data(), np.uint16).reshape(d.get_height(), d.get_width()).copy()
        if raw.shape[::-1] != size: raise ValueError('Depth profile size mismatch')
        rect = cv2.remap(raw, *maps, cv2.INTER_NEAREST)
        width = min(640, size[0]); height = round(size[1]*width/size[0])
        z = cv2.resize(rect, (width, height), interpolation=cv2.INTER_NEAREST).astype(np.float32)*d.get_depth_scale()/1000.
        ks = k.copy(); ks[0,:] *= width/size[0]; ks[1,:] *= height/size[1]
        ex = dp.get_extrinsic_to(cp)
        heat = cv2.applyColorMap(np.uint8(np.clip(z/3.,0,1)*255),cv2.COLORMAP_TURBO); heat[z <= 0] = 0
        try:
            hole = detect_depth_hole(z, ks, (args.min_diameter_mm/1000.,args.max_diameter_mm/1000.))
            cv2.ellipse(heat,hole['ellipse'],(0,255,0),2)
            detection = dict(status='detected',center_depth_m=hole['center'].tolist(),normal_depth=hole['normal'].tolist(),diameter_m=2*hole['radius'],plane_rms_m=hole['plane_rms'],circle_rms_m=hole['circle_rms'])
        except ValueError as exc:
            detection = dict(status='no_hole',reason=str(exc))
        result = dict(camera=info.get_name(),serial=info.get_serial_number(),timestamp_unix=time.time(),depth_size=list(size),depth_scale_mm=d.get_depth_scale(),depth_intrinsics=k.tolist(),detection_intrinsics=ks.tolist(),depth_to_color_sdk_rotation=np.asarray(ex.rot).tolist(),depth_to_color_sdk_translation_mm=np.asarray(ex.transform).tolist(),detection=detection,physical_motion_sent=False,note='SDK factory extrinsic, not robot hand-eye calibration; no robot-frame target computed')
        folder = args.output / time.strftime('orbbec_hole_observation_%Y%m%d_%H%M%S')
        folder.mkdir(parents=True,exist_ok=False)
        for name,image in [('color.png',color_bgr(c)),('depth_raw.png',raw),('detection.png',heat)]:
            if not cv2.imwrite(str(folder/name),image): raise RuntimeError('Image save failed: '+name)
        (folder/'observation.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        print(json.dumps({'output':str(folder),**result},indent=2,ensure_ascii=False),flush=True)
    finally:
        if started: pipe.stop()

if __name__ == '__main__':
    try: main()
    except Exception as e: raise SystemExit(f'采集失败: {e}')
