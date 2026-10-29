#!/usr/bin/env python3
"""Approximate observation view from left feedback; never uses hole ground truth."""
import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from real_hole_capture import Feedback
from real_hole_observe import wait
from real_hole_preview import reset_preview, preview_publisher

ROOT=Path(__file__).resolve().parents[1]

def to_pose(matrix):
    p=PoseStamped();p.header.frame_id='base_link'
    p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,matrix[:3,3])
    q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
    p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,q)
    return p

def matrix_of(transform):
    t=transform.transform;q=t.rotation
    out=np.eye(4);out[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    out[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
    return out

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--distance',type=float,default=.45,help='相机离板大概距离，单位米')
    p.add_argument('--lateral',type=float,default=0.,help='相对估计孔中心沿板面横移，单位米')
    p.add_argument('--aim-offset',type=float,default=0.,help='相机瞄准点相对孔中心沿板面偏移，单位米')
    p.add_argument('--plan-only',action='store_true')
    args=p.parse_args()
    if not .2 <= args.distance <= .6 or not np.isfinite(args.lateral) or abs(args.lateral)>.2 or not np.isfinite(args.aim_offset) or abs(args.aim_offset)>.15:p.error('距离需在 0.2–0.6 m，横移不超过 0.2 m，瞄准偏移不超过 0.15 m')
    rclpy.init(); node=Node('real_hole_auto_observe'); feedback=Feedback()
    buffer=Buffer();listener=TransformListener(buffer,node,spin_thread=False)
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(feedback)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    pub=None
    try:
        deadline=time.monotonic()+10
        while True:
            try:
                q=feedback.stationary(); lg=feedback.gripper_width('left');rg=feedback.gripper_width('right')
                L=matrix_of(buffer.lookup_transform('base_link','left_gripper_base',Time()))
                F=matrix_of(buffer.lookup_transform('base_link','right_flange_link',Time()))
                break
            except Exception:
                if time.monotonic()>deadline:raise
                time.sleep(.1)
        state=RobotState()
        for side in ('left','right'):
            state.joint_state.name.extend([f'{side}_joint{i}' for i in range(1,8)])
            state.joint_state.position.extend(q[side].tolist())
        state.joint_state.name.extend(['left_gripper','right_gripper']);state.joint_state.position.extend([lg,rg])
        ik=node.create_client(GetPositionIK,'/compute_ik');plan=node.create_client(GetMotionPlan,'/plan_kinematic_path')
        if not ik.wait_for_service(timeout_sec=5) or not plan.wait_for_service(timeout_sec=5):raise RuntimeError('MoveIt 规划服务未就绪')
        meta=json.loads((ROOT/'config/handeye/right/d405.json').read_text())
        # Calibration used a flange 1 mm displaced in link7 X relative to the
        # installed official model. Convert that fixed offset into flange axes.
        fixed_r=Rotation.from_euler('xyz',[-1.5708,0,-1.5708]).as_matrix()
        model_to_cal=np.eye(4);model_to_cal[:3,3]=fixed_r.T@np.array([.001,0,0])
        flange_camera=model_to_cal@np.asarray(meta['T_gripper_camera'])
        current_camera=F@flange_camera
        center=(L@np.array([0.,0.,.288,1.]))[:3]
        normal=-L[:3,1];normal/=np.linalg.norm(normal)
        preferred=1 if (current_camera[:3,3]-center)@normal >= 0 else -1
        result=None
        codes={}
        for sign in (preferred,-preferred):
            for offset,dist,roll in [(o,d,r) for d in (args.distance,.30,.55) for o in (args.lateral,args.lateral-.08,args.lateral+.08) for r in (0.,np.pi/2,-np.pi/2,np.pi)]:
                position=center+sign*normal*dist+L[:3,0]*offset
                z=center+L[:3,0]*args.aim_offset-position;z/=np.linalg.norm(z)
                down=np.array([0.,0.,-1.])
                if abs(z@down)>.95:down=np.array([0.,1.,0.])
                x=np.cross(down,z);x/=np.linalg.norm(x);y=np.cross(z,x)
                camera=np.eye(4);camera[:3,:3]=np.column_stack([x,y,z])@Rotation.from_euler('z',roll).as_matrix();camera[:3,3]=position
                goal=camera@np.linalg.inv(flange_camera)
                req=GetPositionIK.Request();req.ik_request.group_name='right_arm';req.ik_request.ik_link_name='right_flange_link'
                req.ik_request.pose_stamped=to_pose(goal);req.ik_request.robot_state=state;req.ik_request.avoid_collisions=True
                req.ik_request.timeout.nanosec=300000000
                solution=wait(ik.call_async(req),10)
                codes[solution.error_code.val]=codes.get(solution.error_code.val,0)+1
                if solution.error_code.val!=1:continue
                vals=dict(zip(solution.solution.joint_state.name,solution.solution.joint_state.position))
                motion=GetMotionPlan.Request();m=motion.motion_plan_request;m.group_name='right_arm';m.pipeline_id='ompl';m.start_state=state
                m.allowed_planning_time=6.;m.num_planning_attempts=3;m.max_velocity_scaling_factor=.05;m.max_acceleration_scaling_factor=.05
                c=Constraints()
                for i in range(1,8):
                    name=f'right_joint{i}';c.joint_constraints.append(JointConstraint(joint_name=name,position=float(vals[name]),tolerance_above=.005,tolerance_below=.005,weight=1.))
                m.goal_constraints=[c]
                candidate=wait(plan.call_async(motion),30).motion_plan_response
                if candidate.error_code.val==1:result=candidate;break
            if result is not None:break
        if result is None:raise RuntimeError(f'没有找到可规划的粗观察位；IK 结果 {codes}，请在 RViz 手动调整右臂')
        pub=preview_publisher(node)
        display=DisplayTrajectory();display.model_id='dual_nero';display.trajectory_start=result.trajectory_start;display.trajectory=[result.trajectory];pub.publish(display)
        print(f'根据左臂反馈和近似板位置规划观察位，预计孔中心 base_link={center.tolist()}；真实孔以相机识别为准。',flush=True)
        print('注意：板姿态是近似，杆和相机的碰撞体未完整建模。轨迹已发布 RViz。',flush=True)
        if args.plan_only:return
        try:answer=input('确认预览后输入 do 执行，其他输入取消：').strip()
        except EOFError:answer=''
        if answer!='do':print('已取消，没有运动。');return
        now=feedback.stationary()
        if any(np.max(np.abs(now[s]-q[s]))>.02 for s in now):raise RuntimeError('双臂状态变化，请重新规划')
        action=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
        if not action.wait_for_server(timeout_sec=5):raise RuntimeError('执行服务不可用')
        goal=ExecuteTrajectory.Goal();goal.trajectory=result.trajectory
        handle=wait(action.send_goal_async(goal),15)
        if not handle.accepted:raise RuntimeError('执行被拒绝')
        result=wait(handle.get_result_async(),120).result
        if result.error_code.val!=1:raise RuntimeError(f'执行失败: {result.error_code.val}')
        print('观察轨迹已执行，请检查右腕相机是否看到孔。',flush=True)
    finally:
        if pub is not None and not args.plan_only:
            try:reset_preview(node,pub)
            except Exception as error:print(f'预览复位未完成：{error}',flush=True)
        executor.shutdown();thread.join();feedback.destroy_node();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
