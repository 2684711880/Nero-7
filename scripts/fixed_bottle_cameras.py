#!/usr/bin/env python3
"""Add approximate visible/collision camera bodies to the fixed-bottle MoveIt scene."""
import math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject,CollisionObject,PlanningScene,ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from inspect_orbbec_hole_target import mount,calibration,ROOT

def transform(xyz=(0,0,0),rpy=(0,0,0)):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();t[:3,3]=xyz;return t

def joint(root,name):
    o=root.find(f"joint[@name='{name}']/origin")
    if o is None:raise ValueError('Missing camera joint '+name)
    return transform(np.fromstring(o.get('xyz','0 0 0'),sep=' '),np.fromstring(o.get('rpy','0 0 0'),sep=' '))

def attach(name,parent,T,size,touch):
    obj=CollisionObject();obj.id=name;obj.header.frame_id=parent;obj.operation=CollisionObject.ADD
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(map(float,size)))]
    p=Pose();p.position.x,p.position.y,p.position.z=map(float,T[:3,3]);q=Rotation.from_matrix(T[:3,:3]).as_quat();p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q);obj.primitive_poses=[p]
    a=AttachedCollisionObject();a.link_name=parent;a.object=obj;a.touch_links=touch;return a

def build():
    root=ET.parse(ROOT/'sim_ws/src/dual_nero_moveit_config/urdf/dual_nero.urdf').getroot();out=[]
    for side in ('left','right'):
        stand=joint(root,side+'_camera_stand_joint')
        body=stand@joint(root,side+'_d435_camera_joint')@joint(root,side+'_camera_joint')@joint(root,side+'_camera_link_joint')@transform([0,-.0175,0])
        touch=[side+'_gripper_base',side+'_gripper_link',side+'_gripper_link1',side+'_gripper_link2',side+'_flange_link',side+'_link7']
        out.append(attach(side+'_wrist_camera_approx',side+'_gripper_base',body,[.02505,.09,.025],touch))
    middle=mount('left')@calibration(ROOT/'archive/handeye/fixed_336l/obzg2.launch.py')@transform([0,0,-.0125])
    out.append(attach('middle_336l_camera_approx','base_link',middle,[.095,.025,.025],['base_link']))
    return out

def main():
    objects=build();rclpy.init();node=Node('fixed_bottle_cameras')
    try:
        c=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        if not c.wait_for_service(timeout_sec=10):raise RuntimeError('MoveIt 场景服务不可用')
        s=PlanningScene();s.is_diff=True;s.robot_state.is_diff=True;s.robot_state.attached_collision_objects=objects
        for obj in objects:
            color=ObjectColor();color.id=obj.object.id;color.color.r=.2;color.color.g=.25;color.color.b=.32;color.color.a=1.;s.object_colors.append(color)
        f=c.call_async(ApplyPlanningScene.Request(scene=s));rclpy.spin_until_future_complete(node,f,timeout_sec=10)
        if not f.done() or f.result() is None or not f.result().success:raise RuntimeError('相机模型未被 MoveIt 接受')
        print('已添加左右腕相机和中间 336L 的近似外形；相机支架未完整建模。',flush=True)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
