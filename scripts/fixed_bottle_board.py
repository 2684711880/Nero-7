#!/usr/bin/env python3
"""Attach a detected approximate plate with an opening to the left gripper; no motor commands."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject,CollisionObject,PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

ROOT=Path(__file__).resolve().parents[1]

def build(target,T_left_base,side=.4,thickness=.008):
    center=np.asarray(target['hole_center_base_m'],dtype=float); n=np.asarray(target['outward_normal_base'],dtype=float)
    diameter=float(target['hole_diameter_mm'])/1000
    if center.shape!=(3,) or n.shape!=(3,) or not np.isfinite(center).all() or not np.isfinite(n).all() or np.linalg.norm(n)<1e-8:raise ValueError('检测到的孔中心或法向无效，请重新采集')
    if not np.isfinite(diameter) or not 0<diameter<=.12:raise ValueError(f'检测孔径 {diameter*1000:.1f} mm 超出孔板模型范围（0 < 孔径 ≤ 120 mm）；请检查孔的检测轮廓并重新采集')
    n/=np.linalg.norm(n);up=np.array([0.,0.,1.]);up-=n*np.dot(up,n)
    if np.linalg.norm(up)<.1:
        up=np.array([0.,1.,0.]);up-=n*np.dot(up,n)
    up/=np.linalg.norm(up);x=np.cross(up,n);R=np.column_stack((x,up,n))
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=center-n*thickness/2
    local=T_left_base@T;inner=diameter/2;half=max(.05,inner+.005);outer=math.sqrt(2)*half
    if half>=side/2:raise ValueError('Detected hole too large for plate construction')
    strips=[([-(side/2+half)/2,0,0],[side/2-half,side,thickness],0),([(side/2+half)/2,0,0],[side/2-half,side,thickness],0),([0,(side/2+half)/2,0],[2*half,side/2-half,thickness],0),([0,-(side/2+half)/2,0],[2*half,side/2-half,thickness],0)]
    count=64;width=2*outer*math.tan(math.pi/count)
    for i in range(count):
        angle=2*math.pi*i/count;radius=(inner+outer)/2
        strips.append(([radius*math.cos(angle),radius*math.sin(angle),0],[outer-inner,width,thickness],angle))
    obj=CollisionObject();obj.id='detected_hole_board';obj.header.frame_id='left_gripper_base';obj.operation=CollisionObject.ADD
    for xyz,size,angle in strips:
        shape=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(map(float,size)))
        pose=Pose();p=(local@np.r_[xyz,1])[:3];quat=Rotation.from_matrix(local[:3,:3]@Rotation.from_euler('z',angle).as_matrix()).as_quat()
        pose.position.x,pose.position.y,pose.position.z=map(float,p);pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,quat)
        obj.primitives.append(shape);obj.primitive_poses.append(pose)
    attached=AttachedCollisionObject();attached.link_name='left_gripper_base';attached.object=obj;attached.touch_links=['left_gripper_base','left_gripper_link1','left_gripper_link2'];return attached

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--targets',type=Path,default=ROOT/'records/fixed_bottle_targets.json');p.add_argument('--side-mm',type=float,default=400.);p.add_argument('--thickness-mm',type=float,default=8.);p.add_argument('--inspection-allow-stale',action='store_true',help='只用于现场确认双臂和板未移动后的只读场景检查');a=p.parse_args()
    if not 150<=a.side_mm<=1000 or not 1<=a.thickness_mm<=50:p.error('Invalid plate size')
    target=json.loads(a.targets.read_text())
    if target.get('source')!='fixed_orbbec_preview':raise ValueError('需要固定相机检测结果')
    if not 0<=time.time()-target['observed_wall_time']<=30:
        if not a.inspection_allow_stale:raise ValueError('需要 30 秒内的固定相机检测结果')
        from handeye_charuco import PassiveJoints
        buses={'left':PassiveJoints('can1')}
        try:
            deadline=time.monotonic()+5
            while True:
                try: now={s:b.snapshot(time.monotonic())[0] for s,b in buses.items()};break
                except ValueError:
                    if time.monotonic()>deadline:raise
                    time.sleep(.1)
            if any(np.max(np.abs(now[s]-np.asarray(target['q_'+s+'_rad'])))>.02 for s in buses):raise ValueError('左臂位置与旧画面时不同，不能使用旧孔板结果')
        finally:
            for b in buses.values():b.close()
        print('仅场景检查：使用用户确认未移动的较早画面；不得用于执行。',flush=True)
    rclpy.init();node=Node('fixed_bottle_board');tf=Buffer();listener=TransformListener(tf,node)
    try:
        end=time.monotonic()+8;trans=None
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            try:trans=tf.lookup_transform('left_gripper_base','base_link',Time()).transform;break
            except Exception:pass
        if trans is None:raise RuntimeError('无法获取左夹爪 TF')
        q=trans.rotation;T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T[:3,3]=[trans.translation.x,trans.translation.y,trans.translation.z]
        scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True;scene.robot_state.attached_collision_objects=[build(target,T,a.side_mm/1000,a.thickness_mm/1000)]
        client=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('MoveIt 场景服务不可用')
        future=client.call_async(ApplyPlanningScene.Request(scene=scene));rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or future.result() is None or not future.result().success:raise RuntimeError('MoveIt 未接受孔板')
        print(f'已更新左臂附着孔板：{a.side_mm:g} mm 方板，厚 {a.thickness_mm:g} mm，检测孔径 {target["hole_diameter_mm"]:.1f} mm；板面内旋转为估计。',flush=True)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
