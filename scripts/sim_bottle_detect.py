#!/usr/bin/env python3
"""NERO Gazebo depth frame -> ArmHub detector; no arm commands."""
import argparse
import json
import sys
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, JointState
from tf2_ros import Buffer, TransformListener

from depth_hole import detect_depth_hole

class Detector(Node):
    def __init__(self,output):
        super().__init__('nero_sim_depth_hole_detector')
        self.output=output;self.image=None;self.info=None;self.joints=None;self.last_stamp=None;self.last_detection_wall=0.
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'joints',m),10)
        self.create_subscription(Image,'/sim/middle/camera/depth/image_raw',self.on_image,10)
        self.create_subscription(CameraInfo,'/sim/middle/camera/depth/camera_info',self.on_info,10)
    def on_image(self,m):self.image=m
    def on_info(self,m):self.info=m
    def detect(self):
        m=self.image
        if m is None or self.info is None or self.joints is None:return None
        if time.monotonic()-self.last_detection_wall < .3:return None
        stamp=(m.header.stamp.sec,m.header.stamp.nanosec)
        if stamp==self.last_stamp:return None
        self.last_stamp=stamp
        self.last_detection_wall=time.monotonic()
        if m.encoding!='32FC1' or m.header.frame_id!='sim_middle_camera_optical':raise ValueError('Unexpected depth encoding or camera frame')
        if (m.width,m.height)!=(self.info.width,self.info.height):raise ValueError('Depth/camera info dimensions differ')
        k=np.array(self.info.k).reshape(3,3);z=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.step//4)[:,:m.width].copy()
        color=cv2.applyColorMap(np.uint8(np.clip(np.nan_to_num(z,nan=2.,posinf=2.)/2,0,1)*255),cv2.COLORMAP_TURBO)
        try:
            h=detect_depth_hole(z,k)
            report=dict(status='detected',frame=m.header.frame_id,center_m=h['center'].tolist(),normal=h['normal'].tolist(),diameter_m=h['radius']*2,plane_rms_m=h['plane_rms'],circle_rms_m=h['circle_rms'],motion_sent=False, image_stamp_sec=float(m.header.stamp.sec+m.header.stamp.nanosec*1e-9), observed_wall_time=time.time(), joint_positions=dict(zip(self.joints.name,self.joints.position)))
            cv2.ellipse(color,h['ellipse'],(0,255,0),2)
            try:
                t=self.tf.lookup_transform(m.header.frame_id,'sim_hole_plate',Time())
                truth=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
                report.update(sim_truth_center_m=truth.tolist(),center_error_mm=float(np.linalg.norm(h['center']-truth)*1000))
            except Exception:pass
        except ValueError as e:report=dict(status='no_hole',reason=str(e),motion_sent=False)
        self.output.mkdir(parents=True,exist_ok=True)
        (self.output/'detection.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        cv2.imwrite(str(self.output/'depth_detection.png'),color)
        return report,color

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=(Path(__file__).resolve().parents[1] / 'sim_ws/validation/bottle_hole/latest'));p.add_argument('--once',action='store_true');p.add_argument('--no-gui',action='store_true');a=p.parse_args()
    rclpy.init();node=Detector(a.output)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
            result=node.detect()
            if result:
                report,image=result;print(json.dumps(report,ensure_ascii=False),flush=True)
                if not a.no_gui:
                    cv2.imshow('NERO simulated depth hole',image)
                    if cv2.waitKey(1)&255 in (ord('q'),27):break
                if a.once:break
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if not a.no_gui:cv2.destroyAllWindows()
if __name__=='__main__':main()
