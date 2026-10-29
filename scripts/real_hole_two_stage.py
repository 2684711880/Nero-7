#!/usr/bin/env python3
"""Preview and explicitly execute approach/feed trajectories from measured targets."""
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
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import BoundingVolume, Constraints, DisplayTrajectory, OrientationConstraint, PositionConstraint, RobotState, PlanningSceneComponents
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPlanningScene
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from real_hole_capture import Feedback
from real_hole_observe import wait
from real_hole_preview import reset_preview, preview_publisher

ROOT = Path(__file__).resolve().parents[1]

def pose(matrix):
    t = np.asarray(matrix, dtype=float)
    if t.shape != (4,4) or not np.isfinite(t).all():
        raise ValueError('invalid target transform')
    q = Rotation.from_matrix(t[:3,:3]).as_quat()
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, t[:3,3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    return p

def scale_trajectory_time(trajectory, velocity_scale):
    """Slow an already time-parameterized joint trajectory without changing its path."""
    scale=float(velocity_scale)
    if not 0 < scale <= 1: raise ValueError('Velocity scale must be in (0, 1]')
    points=trajectory.joint_trajectory.points
    if len(points)<2: raise ValueError('Cartesian trajectory has too few points')
    prev=-1
    for point in points:
        old=int(point.time_from_start.sec)*1_000_000_000+int(point.time_from_start.nanosec)
        if old<=prev: raise ValueError('Cartesian trajectory timestamps must strictly increase')
        prev=old
        new=round(old/scale);point.time_from_start.sec,point.time_from_start.nanosec=divmod(new,1_000_000_000)
        point.velocities=[v*scale for v in point.velocities]
        point.accelerations=[a*scale*scale for a in point.accelerations]
    return trajectory


def confirm(label):
    print(f'{label}轨迹已发布到 RViz。输入 do 执行，其他输入取消。', flush=True)
    try:
        return input('> ').strip() == 'do'
    except EOFError:
        return False

def wait_stationary(feedback, timeout=8.):
    deadline = time.monotonic() + timeout
    while True:
        try:
            return feedback.stationary()
        except RuntimeError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(.1)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--targets', type=Path, default=ROOT/'hardware_ws/validation/real_hole/targets.json')
    parser.add_argument('--left-gripper-mm', type=float, help='左夹爪开口 mm；默认读左臂反馈')
    parser.add_argument('--preview-only', action='store_true', help='只预览两段，不发送轨迹')
    parser.add_argument('--allow-approximate-bottle', action='store_true', help='使用明确显示的瓶子估计参数，仍需先预览并逐段确认')
    parser.add_argument('--inspection-allow-stale', action='store_true', help='仅预览：现场确认孔板未移动后，可检查旧观测，不可执行')
    args = parser.parse_args()
    if args.left_gripper_mm is not None and (not np.isfinite(args.left_gripper_mm) or not 0 <= args.left_gripper_mm <= 100):
        parser.error('left gripper width must be 0..100 mm')
    if args.inspection_allow_stale and not args.preview_only:
        parser.error('--inspection-allow-stale 只能与 --preview-only 一起使用')
    target = json.loads(args.targets.read_text())
    if target.get('status') != 'geometry_only':
        raise ValueError('目标数据无效')
    if not 0 <= time.time()-target.get('observed_wall_time', 0) <= 30:
        if not args.inspection_allow_stale: raise ValueError('观测已过期，请重新采集和生成目标')
        print('仅做碰撞场景检查：观测较早，不能用作实物执行。',flush=True)
    rclpy.init()
    node = Node('nero_real_hole_two_stage')
    feedback = Feedback()
    merged = {}
    def update(m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        for name, value in zip(m.name, m.position):
            merged[name] = (value, stamp)
    subscriber = node.create_subscription(JointState, '/joint_states', update, 1)
    executor = SingleThreadedExecutor()
    executor.add_node(node); executor.add_node(feedback)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    pub=None
    try:
        deadline = time.monotonic()+10
        while True:
            try:
                q = feedback.stationary()
                if 'right_gripper' not in merged: raise RuntimeError('waiting for right gripper')
                break
            except RuntimeError:
                if time.monotonic() > deadline: raise
                time.sleep(.1)
        for side in ('left','right'):
            if np.max(np.abs(q[side]-np.asarray(target[f'q_{side}_rad']))) > .02:
                raise RuntimeError('双臂位置与采集时不同，重新采集')
        grip = merged['right_gripper']
        if not 0 <= time.time()-grip[1] < .3: raise RuntimeError('右夹爪反馈已过期')
        state = RobotState()
        for side in ('left','right'):
            state.joint_state.name.extend([f'{side}_joint{i}' for i in range(1,8)])
            state.joint_state.position.extend(q[side].tolist())
        state.joint_state.name.extend(['left_gripper','right_gripper'])
        left_width = feedback.gripper_width('left') if args.left_gripper_mm is None else args.left_gripper_mm / 1000.
        state.joint_state.position.extend([left_width,grip[0]])
        state.is_diff = False
        scene_client=node.create_client(GetPlanningScene, '/get_planning_scene')
        if not scene_client.wait_for_service(timeout_sec=5): raise RuntimeError('MoveIt 场景服务不可用')
        scene_req=GetPlanningScene.Request()
        scene_req.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        scene=wait(scene_client.call_async(scene_req),5).scene
        state.attached_collision_objects=list(scene.robot_state.attached_collision_objects)
        attached_ids=[o.object.id for o in state.attached_collision_objects]
        if target.get('source')=='fixed_orbbec_preview' and 'detected_hole_board' not in attached_ids:
            raise RuntimeError('固定相机孔板没有加入规划场景；请先更新孔板')
        print('规划起点附着物:', attached_ids, flush=True)
        planner = node.create_client(GetMotionPlan,'/plan_kinematic_path')
        cart = node.create_client(GetCartesianPath,'/compute_cartesian_path')
        if not planner.wait_for_service(timeout_sec=5) or not cart.wait_for_service(timeout_sec=5):
            raise RuntimeError('MoveIt 规划服务不可用')
        approach_base=np.asarray(target['T_base_flange_approach'],float)
        feed_base=np.asarray(target['T_base_flange_feed'],float)
        tip=np.asarray(target['tool_tip_flange_mm'],float)/1000.
        normal=np.asarray(target['outward_normal_base'],float);normal/=np.linalg.norm(normal)
        offsets=[0.,45.,-45.,90.,-90.,135.,-135.,180.] if target.get('source')=='fixed_orbbec_preview' else [0.]
        response=feed=end=path=None
        for angle in offsets:
            def rotated(original):
                updated=original.copy()
                fixed_tip=original[:3,3]+original[:3,:3]@tip
                updated[:3,:3]=Rotation.from_rotvec(np.deg2rad(angle)*normal).as_matrix()@original[:3,:3]
                updated[:3,3]=fixed_tip-updated[:3,:3]@tip
                return updated
            approach_pose,feed_pose=pose(rotated(approach_base)),pose(rotated(feed_base))
            req=GetMotionPlan.Request();m=req.motion_plan_request
            m.group_name='right_arm';m.pipeline_id='ompl';m.start_state=state
            m.allowed_planning_time=4.;m.num_planning_attempts=3
            m.max_velocity_scaling_factor=.05;m.max_acceleration_scaling_factor=.05
            pc=PositionConstraint();pc.header.frame_id='base_link';pc.link_name='right_flange_link';pc.weight=1.
            pc.constraint_region=BoundingVolume();pc.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.002])];pc.constraint_region.primitive_poses=[approach_pose]
            oc=OrientationConstraint();oc.header.frame_id='base_link';oc.link_name='right_flange_link';oc.orientation=approach_pose.orientation
            oc.absolute_x_axis_tolerance=oc.absolute_y_axis_tolerance=oc.absolute_z_axis_tolerance=.02;oc.weight=1.
            m.goal_constraints=[Constraints(position_constraints=[pc],orientation_constraints=[oc])]
            print(f'检查瓶轴周围朝向 {angle:+g}°…',flush=True)
            candidate=wait(planner.call_async(req),20).motion_plan_response
            if candidate.error_code.val!=1:continue
            candidate_path=candidate.trajectory.joint_trajectory
            end_positions=dict(zip(state.joint_state.name,state.joint_state.position));end_positions.update(zip(candidate_path.joint_names,candidate_path.points[-1].positions))
            candidate_end=RobotState();candidate_end.joint_state.name=list(end_positions);candidate_end.joint_state.position=list(end_positions.values());candidate_end.attached_collision_objects=list(state.attached_collision_objects)
            cr=GetCartesianPath.Request();cr.header.frame_id='base_link';cr.start_state=candidate_end;cr.group_name='right_arm';cr.link_name='right_flange_link';cr.waypoints=[feed_pose];cr.max_step=.002;cr.jump_threshold=0.;cr.avoid_collisions=True
            candidate_feed=wait(cart.call_async(cr),45)
            if candidate_feed.fraction<.999:continue
            scale_trajectory_time(candidate_feed.solution, .05)
            last=candidate_feed.solution.joint_trajectory.points[-1].time_from_start
            print(f'前送轨迹按 5% 缩放，时长 {last.sec+last.nanosec/1e9:.2f} 秒。',flush=True)
            response,path,end,feed=candidate,candidate_path,candidate_end,candidate_feed
            print(f'已找到完整的孔前和前送规划，瓶轴转角 {angle:+g}°。',flush=True)
            break
        if response is None:raise RuntimeError('不同瓶轴朝向均未找到完整的碰撞检查路径；没有发送动作')
        pub=preview_publisher(node)
        action=ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
        def display(trajectory, start):
            msg=DisplayTrajectory(); msg.model_id='dual_nero'; msg.trajectory_start=start; msg.trajectory=[trajectory]; pub.publish(msg)
        def send(trajectory):
            if not action.wait_for_server(timeout_sec=5): raise RuntimeError('执行服务不可用')
            goal=ExecuteTrajectory.Goal(); goal.trajectory=trajectory
            handle=wait(action.send_goal_async(goal),15)
            if not handle.accepted: raise RuntimeError('轨迹被拒绝')
            if wait(handle.get_result_async(),120).result.error_code.val != 1: raise RuntimeError('轨迹执行失败')
        print('注意：孔板为估计尺寸和姿态，瓶子及相机的碰撞体尚未完整包含。',flush=True)
        if args.preview_only:
            display_msg=DisplayTrajectory(); display_msg.model_id='dual_nero'; display_msg.trajectory_start=response.trajectory_start
            display_msg.trajectory=[response.trajectory, feed.solution]; pub.publish(display_msg)
            print('两段轨迹已发布 RViz，仅预览，没有发送运动。',flush=True)
            print(target.get('warning', ''),flush=True)
            time.sleep(2.)
            return
        if not target.get('tool_geometry_supplied'):
            if not (args.allow_approximate_bottle and target.get('source')=='fixed_orbbec_preview'):
                raise RuntimeError('工具几何未完成；仅允许 --preview-only')
            print('使用估计瓶底及瓶身参数；请按实物检查位置、姿态和全部空间，确认后再输入 do。',flush=True)
        display(response.trajectory,response.trajectory_start)
        if not confirm('孔前'): return
        now=feedback.stationary()
        if time.time()-target['observed_wall_time']>120 or any(np.max(np.abs(now[s]-q[s]))>.02 for s in now):
            raise RuntimeError('观测过期或双臂已移动，重新采集规划')
        send(response.trajectory)
        actual=wait_stationary(feedback)
        expected=dict(zip(path.joint_names,path.points[-1].positions))
        if np.max(np.abs(actual['left']-q['left']))>.02 or any(abs(actual['right'][i-1]-expected[f'right_joint{i}'])>.02 for i in range(1,8)):
            raise RuntimeError('孔前位置或左臂反馈发生变化，不执行前送')
        display(feed.solution,end)
        if not confirm('前送'): return
        now=feedback.stationary()
        if time.time()-target['observed_wall_time']>120 or any(np.max(np.abs(now[s]-actual[s]))>.02 for s in now):
            raise RuntimeError('观测过期或双臂已移动，不执行前送')
        send(feed.solution)
        print('控制器报告前送完成；未验证实物穿孔。',flush=True)
    finally:
        if pub is not None and not args.preview_only:
            try:reset_preview(node,pub)
            except Exception as error:print(f'预览复位未完成：{error}',flush=True)
        executor.shutdown(); thread.join(); feedback.destroy_node(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
