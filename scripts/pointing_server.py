#!/usr/bin/env python3
"""RViz corner menu: observe/plan, replay, then explicitly execute the saved plan."""
import argparse
import copy
from array import array as ByteArray
from collections import deque
import json
from pathlib import Path
import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from moveit_msgs.msg import (DisplayTrajectory, Constraints, PositionConstraint, OrientationConstraint,
                             BoundingVolume, CollisionObject, RobotState, PlanningScene)
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene
from moveit_msgs.action import ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from rosidl_runtime_py.convert import message_to_ordereddict
from handeye_charuco import ROOT, FK, Board, write_json
from handeye_rviz_inputs import quaternion
from point_charuco import pointing_target, select_corner
from pointing_model import mount_matrix


def pose_of(t):
    p=Pose()
    p.position.x,p.position.y,p.position.z=map(float,t[:3,3])
    p.orientation=Quaternion(**dict(zip(('x','y','z','w'),quaternion(t[:3,:3]))))
    return p


def gravity_workers(channel):
    """Identify the currently used official gravity worker; never stop it."""
    found=[]
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:args=path.read_bytes().decode().split('\0')
        except (OSError,UnicodeError):continue
        if not any(Path(a).name=='nero_gc_official_configured.py' for a in args):continue
        side=args[args.index('--side')+1] if '--side' in args else 'right'
        if channel==('can1' if side=='left' else 'can0'):found.append(int(path.parent.name))
    return found


def wait(future,timeout=20.):
    done=threading.Event();future.add_done_callback(lambda _:done.set())
    if not done.wait(timeout):raise RuntimeError('ROS request timed out')
    return future.result()


class Pointing(Node):
    def __init__(self,args):
        super().__init__('pointing')
        self.args=args;self.group=ReentrantCallbackGroup();self.busy=threading.Lock()
        self.meta=json.loads((ROOT/'config/handeye/can1_d405.json').read_text())
        if self.meta['can']!=args.can:raise ValueError('Calibration belongs to a different CAN interface')
        self.x=np.array(self.meta['T_gripper_camera'])
        self.fk=FK(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf')
        self.mount=mount_matrix('left')
        self.board=Board(json.loads((ROOT/'config/handeye_charuco.json').read_text()))
        self.corner=select_corner(self.board,'board')
        self.image=self.info=self.joint=None;self.history=deque(maxlen=150)
        self.positions={};self.arm_stamp={}
        self.trajectory=None;self.goal_handle=None;self.report=None;self.capture_q=None
        self.create_subscription(Image,'/handeye/color/image_raw',lambda m:setattr(self,'image',m),5,callback_group=self.group)
        self.create_subscription(CameraInfo,'/handeye/color/camera_info',lambda m:setattr(self,'info',m),5,callback_group=self.group)
        self.create_subscription(JointState,'/joint_states',self.on_joint,10,callback_group=self.group)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers=self.create_publisher(MarkerArray,'/pointing/markers',qos)
        self.display=self.create_publisher(DisplayTrajectory,'/display_planned_path',qos)
        self.annotated=self.create_publisher(Image,'/pointing/target_image',10)
        self.planner=self.create_client(GetMotionPlan,'/plan_kinematic_path',callback_group=self.group)
        self.scene=self.create_client(ApplyPlanningScene,'/apply_planning_scene',callback_group=self.group)
        self.execute_client=ActionClient(self,ExecuteTrajectory,'/execute_trajectory',callback_group=self.group)
        self.server=InteractiveMarkerServer(self,'pointing/menu')
        self.menu=MenuHandler()
        self.menu.insert('1 Observe board and plan',callback=lambda _:self.dispatch(self.plan))
        self.menu.insert('2 Replay saved trajectory',callback=lambda _:self.dispatch(self.replay))
        self.menu.insert('3 EXECUTE saved trajectory',callback=lambda _:self.dispatch(self.execute))
        self.menu.insert('4 Cancel execution',callback=lambda _:self.cancel())
        for name,fn in [('plan',self.plan),('replay',self.replay),('execute',self.execute)]:
            self.create_service(Trigger,'/pointing/'+name,lambda r,s,fn=fn:self.service(fn,s),callback_group=self.group)
        self.publish_menu([-.2,0.,.4])
        if args.mock:
            self.joint_pub=self.create_publisher(JointState,'/joint_states',10)
            self.create_timer(.02,self.mock_feedback,callback_group=self.group)
        self.status('Ready. Right-click the red sphere: observe/plan -> replay -> execute.')

    def status(self,text):
        self.get_logger().info(text)
        write_json(self.args.session/'status.json',dict(message=text,time=time.time(),mock=self.args.mock))

    def dispatch(self,fn):
        # Work outside interactive-marker callbacks so camera and service replies keep flowing.
        threading.Thread(target=lambda:self.service(fn,Trigger.Response()),daemon=True).start()

    def service(self,fn,response):
        if not self.busy.acquire(blocking=False):
            response.success=False;response.message='Operation already running';return response
        try:
            response.message=fn();response.success=True
        except Exception as exc:
            response.success=False;response.message=str(exc)
        finally:self.busy.release()
        self.status(response.message)
        return response

    def mock_feedback(self):
        m=JointState();m.header.stamp=self.get_clock().now().to_msg()
        m.name=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
        m.position=self.args.initial_q+self.args.initial_q
        self.joint_pub.publish(m)

    def on_joint(self,m):
        d=dict(zip(m.name,m.position))
        if not np.isfinite(list(d.values())).all():return
        self.positions.update(d)
        now=time.monotonic()
        for side in ('left','right'):
            if all(f'{side}_joint{i}' in d for i in range(1,8)):self.arm_stamp[side]=now
        names=[f'{s}_joint{i}' for s in ('left','right') for i in range(1,8)]
        if not all(n in self.positions for n in names):return
        state=JointState();state.header=m.header;state.name=names
        state.position=[self.positions[n] for n in names];self.joint=state
        if all(f'left_joint{i}' in d for i in range(1,8)):
            self.history.append((now,np.array([d[f'left_joint{i}'] for i in range(1,8)])))

    def base_flange(self,q):
        return self.mount@self.fk(q)

    def current_q(self,stationary=False):
        history=list(self.history)
        if any(time.monotonic()-self.arm_stamp.get(s,0)>.15 for s in ('left','right')):
            raise ValueError('Both arms need fresh feedback for dual-arm collision checking')
        if not history or time.monotonic()-history[-1][0]>.15:
            raise ValueError('Missing fresh seven-joint feedback')
        if stationary:
            recent=[q for t,q in history if time.monotonic()-t<.6]
            if len(recent)<10 or np.max(np.ptp(recent,axis=0))>np.deg2rad(.3):
                raise ValueError('Keep arm stationary for 0.6 seconds, then plan')
        return history[-1][1].copy()

    def observe(self,q):
        if self.args.mock:
            # A small reachable displacement; synthetic target is visibly labelled MOCK.
            bg=self.base_flange(q);bc=bg@self.x
            desired=(bg@np.r_[np.array(self.args.tcp_offset_mm)/1000.,1.])[:3]+[.01,0,0]
            bb=np.eye(4);bb[:3,:3]=bg[:3,:3]@np.diag([1.,-1.,-1.])
            board_corner=desired+self.args.standoff*bg[:3,2]
            bb[:3,3]=board_corner-bb[:3,:3]@self.corner
            return np.linalg.inv(bc)@bb,None,None
        im,info=self.image,self.info
        if im is None or info is None:raise ValueError('Waiting for camera RGB/CameraInfo')
        age=time.time()-im.header.stamp.sec-im.header.stamp.nanosec*1e-9
        if not 0<=age<.3:raise ValueError('Camera image is stale')
        if im.header.frame_id!=self.meta['camera_frame']:raise ValueError('Camera frame mismatch')
        image=np.frombuffer(im.data,dtype=np.uint8).reshape(im.height,im.step)[:,:im.width*3].reshape(im.height,im.width,3).copy()
        k=np.array(info.k).reshape(3,3);d=np.array(info.d)
        det,preview=self.board.detect(image,k,d)
        if det is None or det['reprojection_rms_px']>1.5:raise ValueError('Board not detected reliably')
        cb=np.array(det['T_camera_board'])
        uv=cv2.projectPoints(self.corner.reshape(1,3),cv2.Rodrigues(cb[:3,:3])[0],cb[:3,3],k,d)[0].reshape(2)
        cv2.circle(preview,tuple(uv.astype(int)),10,(0,0,255),3)
        cv2.putText(preview,'TARGET: portrait board TOP LEFT',(10,30),
                    cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,255),2)
        msg=Image();msg.header=im.header;msg.height,msg.width=preview.shape[:2]
        msg.encoding='bgr8';msg.step=msg.width*3;msg.data=ByteArray('B',preview.tobytes())
        self.annotated.publish(msg)
        cv2.imwrite(str(self.args.session/'target.png'),preview)
        return cb,det,msg

    def publish_menu(self,position):
        im=InteractiveMarker();im.header.frame_id='base_link';im.name='board_corner'
        im.description='Right click: Plan / Replay / Execute'+(' [MOCK]' if self.args.mock else '')
        im.scale=.15;im.pose.position.x,im.pose.position.y,im.pose.position.z=map(float,position)
        im.pose.orientation.w=1.
        ctrl=InteractiveMarkerControl();ctrl.interaction_mode=InteractiveMarkerControl.MENU;ctrl.always_visible=True
        sphere=Marker();sphere.type=Marker.SPHERE;sphere.pose.orientation.w=1.
        sphere.scale.x=sphere.scale.y=sphere.scale.z=.025;sphere.color.r=1.;sphere.color.a=1.
        ctrl.markers=[sphere];im.controls=[ctrl]
        self.server.insert(im);self.menu.apply(self.server,im.name);self.server.applyChanges()

    def publish_geometry(self,bb,report):
        msg=MarkerArray()
        def marker(kind,idx):
            m=Marker();m.header.frame_id='base_link';m.header.stamp=self.get_clock().now().to_msg()
            m.ns='pointing';m.id=idx;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;m.color.a=1.
            msg.markers.append(m);return m
        outline=marker(Marker.LINE_STRIP,0);outline.scale.x=.003;outline.color.b=1.;outline.color.g=.7
        for p in ([0,0,0],[.28,0,0],[.28,.20,0],[0,.20,0],[0,0,0]):
            v=(bb@np.r_[p,1.])[:3];outline.points.append(Point(x=float(v[0]),y=float(v[1]),z=float(v[2])))
        tip=marker(Marker.SPHERE,1);tip.scale.x=tip.scale.y=tip.scale.z=.015;tip.color.g=1.
        tip.pose.position.x,tip.pose.position.y,tip.pose.position.z=map(float,report['target_tcp_base_m'])
        line=marker(Marker.LINE_STRIP,2);line.scale.x=.003;line.color.r=line.color.g=1.
        for v in [report['corner_base_m'],report['target_tcp_base_m']]:line.points.append(Point(x=v[0],y=v[1],z=v[2]))
        text=marker(Marker.TEXT_VIEW_FACING,3);text.scale.z=.025;text.color.r=text.color.g=text.color.b=1.
        text.pose.position=copy.deepcopy(tip.pose.position);text.pose.position.z+=.04
        text.text='MODEL gripper center goal'+(' [MOCK]' if self.args.mock else '')
        actual=marker(Marker.SPHERE,4);actual.header.frame_id='left_point_tcp';actual.frame_locked=True
        actual.scale.x=actual.scale.y=actual.scale.z=.012;actual.color.r=1.;actual.color.g=.5
        self.markers.publish(msg);self.publish_menu(report['corner_base_m'])

    def plan(self):
        self.trajectory=None
        q=self.current_q(stationary=True)
        cb,det,_=self.observe(q)
        bg=self.base_flange(q);bb=bg@self.x@cb
        report=pointing_target(bg,self.x,cb,self.corner,self.args.standoff,np.array(self.args.tcp_offset_mm)/1000.)
        self.publish_geometry(bb,report)
        if not self.planner.wait_for_service(timeout_sec=5.):raise RuntimeError('MoveIt planner not ready')
        # Include the observed board as a thin collision box, with its centre transformed correctly.
        obj=CollisionObject();obj.header.frame_id='base_link';obj.id='charuco_board';obj.operation=CollisionObject.ADD
        shape=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.28,.20,.004]);obj.primitives=[shape]
        center=bb.copy();center[:3,3]=(bb@np.array([.14,.10,0,1.]))[:3];obj.primitive_poses=[pose_of(center)]
        if not self.scene.wait_for_service(timeout_sec=3.):raise RuntimeError('Planning scene unavailable')
        scene=ApplyPlanningScene.Request();scene.scene.is_diff=True;scene.scene.world.collision_objects=[obj]
        if not wait(self.scene.call_async(scene)).success:raise RuntimeError('Board collision scene rejected')
        req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='left_arm';r.pipeline_id='ompl'
        r.allowed_planning_time=5.;r.num_planning_attempts=3
        r.max_velocity_scaling_factor=.10;r.max_acceleration_scaling_factor=.10
        r.start_state.joint_state=self.joint;r.start_state.is_diff=False
        pc=PositionConstraint();pc.header.frame_id='base_link';pc.link_name='left_point_tcp';pc.weight=1.
        pc.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.002])]
        goal=np.eye(4);goal[:3,:3]=bg[:3,:3];goal[:3,3]=report['target_tcp_base_m']
        pc.constraint_region.primitive_poses=[pose_of(goal)]
        oc=OrientationConstraint();oc.header.frame_id='base_link';oc.link_name='left_point_tcp';oc.weight=1.
        oc.orientation=pose_of(goal).orientation
        oc.absolute_x_axis_tolerance=oc.absolute_y_axis_tolerance=oc.absolute_z_axis_tolerance=.02
        constraints=Constraints();constraints.position_constraints=[pc];constraints.orientation_constraints=[oc]
        r.goal_constraints=[constraints]
        self.status('Planning from measured seven-joint state; no physical motion.')
        response=wait(self.planner.call_async(req),timeout=25.).motion_plan_response
        if response.error_code.val!=1:raise RuntimeError(f'MoveIt planning failed: code={response.error_code.val}')
        self.trajectory=response.trajectory;self.start=response.trajectory_start
        self.capture_q=q;self.report=report;self.planned_at=time.monotonic()
        self.right_capture=np.array([self.positions[f'right_joint{i}'] for i in range(1,8)])
        report.update(mock=self.args.mock,can=self.args.can,tcp_offset_mm=self.args.tcp_offset_mm,
                      corner_board_m=self.corner.tolist(),corner_definition='printed_portrait_top_left',
                      q_start_rad=q.tolist(),standoff_m=self.args.standoff,motion_sent=False,
                      trajectory=message_to_ordereddict(self.trajectory))
        write_json(self.args.session/'planned_target.json',report)
        self.replay()
        return 'Plan ready: ghost robot plays the saved trajectory. Inspect red corner/green TCP goal, then explicitly Execute.'

    def replay(self):
        if self.trajectory is None:raise ValueError('Observe and plan first')
        display=DisplayTrajectory();display.model_id='dual_nero_pointing'
        display.trajectory_start=self.start;display.trajectory=[self.trajectory];self.display.publish(display)
        return 'Replaying the saved plan; no motion command sent.'

    def execute(self):
        if self.args.mock:raise ValueError('MOCK mode cannot execute physical motion')
        if self.trajectory is None:raise ValueError('No successful plan to execute')
        workers=gravity_workers(self.args.can)
        if workers:raise ValueError(f'Stop gravity compensation yourself first, then Execute. Active PID(s): {workers}')
        q=self.current_q(stationary=True)
        if np.max(np.abs(q-self.capture_q))>.02:raise ValueError('Arm moved since planning; Observe and plan again')
        right=np.array([self.positions[f'right_joint{i}'] for i in range(1,8)])
        if np.max(np.abs(right-self.right_capture))>.02:raise ValueError('Other arm moved since collision checking; replan')
        if time.monotonic()-self.planned_at>300:raise ValueError('Plan older than 5 minutes; Observe and plan again')
        # Reject a moved board as well as a moved arm.
        cb,_,_=self.observe(q)
        current=pointing_target(self.base_flange(q),self.x,cb,self.corner,self.args.standoff,np.array(self.args.tcp_offset_mm)/1000.)
        if np.linalg.norm(np.array(current['corner_base_m'])-self.report['corner_base_m'])>.01:
            raise ValueError('Board observation changed >10mm; replan')
        if not self.execute_client.wait_for_server(timeout_sec=5.):raise RuntimeError('Official execution stack unavailable')
        goal=ExecuteTrajectory.Goal();goal.trajectory=self.trajectory
        self.goal_handle=wait(self.execute_client.send_goal_async(goal))
        if not self.goal_handle.accepted:raise RuntimeError('MoveIt rejected execution')
        write_json(self.args.session/'execution.json',dict(motion_sent=True,reached_by_feedback=False,state='accepted'))
        self.status('Executing the same saved trajectory via official FollowJointTrajectory / ROS / SDK flow.')
        duration=self.trajectory.joint_trajectory.points[-1].time_from_start
        timeout=duration.sec+duration.nanosec*1e-9+15.
        try:result=wait(self.goal_handle.get_result_async(),timeout=max(30.,timeout)).result
        except Exception:
            self.goal_handle.cancel_goal_async()
            raise
        if result.error_code.val!=1:raise RuntimeError(f'Execution failed: {result.error_code.val}')
        names=self.trajectory.joint_trajectory.joint_names
        target=dict(zip(names,self.trajectory.joint_trajectory.points[-1].positions))
        wanted=np.array([target[f'left_joint{i}'] for i in range(1,8)])
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            measured=self.current_q()
            if np.max(np.abs(measured-wanted))<.01:
                write_json(self.args.session/'execution.json',dict(motion_sent=True,reached_by_feedback=True,
                    q_final_rad=measured.tolist(),max_joint_error_rad=float(np.max(np.abs(measured-wanted)))))
                self.trajectory=None
                return 'Execution finished; all seven joints confirmed within 0.01rad. Physically measure pointing error.'
            time.sleep(.05)
        raise RuntimeError('Controller finished but physical seven-joint arrival was not confirmed')

    def cancel(self):
        if self.goal_handle:self.goal_handle.cancel_goal_async()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--can',required=True);p.add_argument('--tcp-offset-mm',type=float,nargs=3,required=True)
    p.add_argument('--standoff',type=float,default=.10);p.add_argument('--session',type=Path,required=True)
    p.add_argument('--mock',action='store_true');p.add_argument('--initial-q',type=float,nargs=7,default=[0,1.2,0,.3,0,0,0])
    args,rosargs=p.parse_known_args()
    args.session.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=rosargs);node=Pointing(args);executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        node.server.shutdown();executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
