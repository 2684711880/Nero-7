#!/usr/bin/env python3
"""Add an approximate 400 mm board with a 90 mm hole to the left gripper."""
import math
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

def build():
    obj = CollisionObject()
    obj.id = 'left_held_board_approx'
    obj.header.frame_id = 'left_gripper_base'
    obj.operation = CollisionObject.ADD
    base = Rotation.from_euler('x', math.pi/2).as_matrix()
    center = np.array([0., 0., .288])
    boxes = [([-.125,0,0],[.15,.4,.008],0),([.125,0,0],[.15,.4,.008],0),
             ([0,.125,0],[.1,.15,.008],0),([0,-.125,0],[.1,.15,.008],0)]
    n=32; inner=.045; outer=math.sqrt(2)*.05; width=2*outer*math.tan(math.pi/n)
    for i in range(n):
        angle=2*math.pi*i/n; rad=(inner+outer)/2
        boxes.append(([rad*math.cos(angle),rad*math.sin(angle),0],[outer-inner,width,.008],angle))
    for xyz,size,angle in boxes:
        shape=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=size)
        q=Rotation.from_matrix(base @ Rotation.from_euler('z',angle).as_matrix()).as_quat()
        position=center+base@np.asarray(xyz)
        p=Pose();p.position.x,p.position.y,p.position.z=map(float,position)
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)
        obj.primitives.append(shape);obj.primitive_poses.append(p)
    attached=AttachedCollisionObject()
    attached.link_name='left_gripper_base'
    attached.object=obj
    attached.touch_links=['left_gripper_base','left_gripper_link1','left_gripper_link2']
    return attached

def main():
    rclpy.init(); node=Node('nero_left_board_scene')
    try:
        client=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('MoveIt 场景服务不可用')
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True
        scene.robot_state.attached_collision_objects=[build()]
        f=client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(node,f,timeout_sec=10)
        if not f.done() or f.result() is None or not f.result().success:
            raise RuntimeError('MoveIt 未接受板模型')
        print('已添加左夹爪持板模型：400×400 mm、孔径90 mm；夹持位置为仿真近似。',flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    main()
