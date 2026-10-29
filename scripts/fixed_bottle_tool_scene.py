#!/usr/bin/env python3
"""Add approximate held-bottle collision geometry to preview MoveIt; no motor commands."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject,CollisionObject,PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
ROOT=Path(__file__).resolve().parents[1]

def build(cfg, radius, length, neck_radius, neck_length):
    grip=np.asarray(cfg['grasp_center_from_flange_m'],float);axis=np.asarray(cfg['bottle_axis_from_grasp_to_bottom'],float);distance=float(cfg['grasp_to_bottom_m'])
    if grip.shape!=(3,) or axis.shape!=(3,) or not np.isfinite(grip).all() or not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-8 or not 0<distance<length or not 0<neck_radius<=radius<.05 or not 0<neck_length<.2:raise ValueError('瓶子模型参数无效')
    axis/=np.linalg.norm(axis);tip=grip+axis*distance;body_center=tip-axis*length/2;neck_center=tip-axis*(length+neck_length/2)
    rot,_=Rotation.align_vectors([axis],[[0,0,1]]);quat=rot.as_quat()
    obj=CollisionObject();obj.id='right_held_bottle_approx';obj.header.frame_id='right_flange_link';obj.operation=CollisionObject.ADD
    for center,r,h in [(body_center,radius,length),(neck_center,neck_radius,neck_length)]:
        shape=SolidPrimitive(type=SolidPrimitive.CYLINDER,dimensions=[float(h),float(r)]);pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,center);pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,quat);obj.primitives.append(shape);obj.primitive_poses.append(pose)
    attached=AttachedCollisionObject();attached.link_name='right_flange_link';attached.object=obj;attached.touch_links=['right_flange_link','right_gripper_base','right_gripper_link','right_gripper_link1','right_gripper_link2'];return attached

def main():
    p=argparse.ArgumentParser();p.add_argument('--tool',type=Path,default=ROOT/'config/bottle_tool_preview.json');p.add_argument('--diameter-mm',type=float,default=56.);p.add_argument('--length-mm',type=float,default=150.);p.add_argument('--neck-diameter-mm',type=float,default=30.);p.add_argument('--neck-length-mm',type=float,default=40.);a=p.parse_args();obj=build(json.loads(a.tool.read_text()),a.diameter_mm/2000,a.length_mm/1000,a.neck_diameter_mm/2000,a.neck_length_mm/1000)
    rclpy.init();n=Node('fixed_bottle_tool_scene')
    try:
        c=n.create_client(ApplyPlanningScene,'/apply_planning_scene')
        if not c.wait_for_service(timeout_sec=5):raise RuntimeError('MoveIt 场景不可用')
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True;scene.robot_state.attached_collision_objects=[obj];f=c.call_async(ApplyPlanningScene.Request(scene=scene));rclpy.spin_until_future_complete(n,f,timeout_sec=10)
        if not f.done() or f.result() is None or not f.result().success:raise RuntimeError('瓶子碰撞体未被接受')
        print(f'已添加右夹爪持瓶模型：瓶身直径 {a.diameter_mm:g} mm，长度 {a.length_mm:g} mm；均为仿真估计。',flush=True)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
