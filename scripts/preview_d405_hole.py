#!/usr/bin/env python3
"""Real D405 depth-only hole detection preview; no robot connection."""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from depth_hole import detect_depth_hole


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial', default='262622274345')
    args = parser.parse_args()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    profile = pipeline.start(config)
    try:
        intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().intrinsics
        if any(abs(c) > 1e-8 for c in intr.coeffs):
            raise RuntimeError('Depth stream has nonzero distortion; cannot apply pinhole detector')
        k = np.array([[intr.fx, 0., intr.ppx], [0., intr.fy, intr.ppy], [0., 0., 1.]])
        print(f'D405 {args.serial}: native depth 640x480, press q to close; no arm connection.', flush=True)
        cv2.namedWindow('D405 depth hole detection', cv2.WINDOW_NORMAL)
        cv2.namedWindow('D405 color', cv2.WINDOW_NORMAL)
        while True:
            frames = pipeline.wait_for_frames(5000)
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue
            metres = np.asanyarray(depth.get_data()).astype(np.float32) * depth.get_units()
            image = cv2.applyColorMap(np.uint8(np.clip(metres / 1.0, 0, 1) * 255), cv2.COLORMAP_TURBO)
            image[metres <= 0] = (0, 0, 0)
            try:
                result = detect_depth_hole(metres, k)
                cv2.ellipse(image, result['ellipse'], (0, 255, 0), 2)
                text = f"Hole diameter {result['radius']*2000:.1f} mm, depth {result['center'][2]*1000:.0f} mm"
                colour = (0, 255, 0)
            except ValueError:
                text, colour = 'No hole detected', (0, 0, 255)
            cv2.putText(image, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, colour, 2)
            cv2.putText(image, 'Black = no depth; q = quit', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
            cv2.imshow('D405 depth hole detection', image)
            cv2.imshow('D405 color', cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 255 in (ord('q'), 27):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
