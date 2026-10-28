#!/usr/bin/env python3
"""Manual RGB-D/CAN capture and offline RegHEC eye-in-hand refinement."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import shlex
import subprocess
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree

from handeye_charuco import ROOT, FK, PassiveJoints, write_json

INITIAL = ROOT/'config/handeye/small2_13_candidate.json'
EXE = ROOT/'third_party/reghec_build/reghec'
LIB = ROOT/'third_party/reghec_deps/root/usr/lib/x86_64-linux-gnu'


def depth_to_color(extrinsics):
    """RealSense rotation is stored column-major; result maps depth to color."""
    t = np.eye(4)
    t[:3,:3] = np.asarray(extrinsics.rotation, float).reshape(3,3,order='F')
    t[:3,3] = np.asarray(extrinsics.translation, float)
    if not np.isfinite(t).all() or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-4):
        raise ValueError('Invalid RealSense depth-to-color extrinsics')
    return t


def select_points(vertices, roi, transform, z_min, z_max, max_points=18000):
    """Select valid depth-camera XYZ within a depth-image rectangle, in color frame."""
    x,y,w,h=map(int,roi)
    if w<=0 or h<=0 or x<0 or y<0 or y+h>vertices.shape[0] or x+w>vertices.shape[1]:
        raise ValueError('Invalid object rectangle')
    pts=vertices[y:y+h,x:x+w].reshape(-1,3)
    pts=pts[np.isfinite(pts).all(axis=1)&(pts[:,2]>=z_min)&(pts[:,2]<=z_max)]
    if len(pts)<300:
        raise ValueError(f'Only {len(pts)} valid points in selection; select object and check depth')
    if len(pts)>max_points:
        pts=pts[np.random.default_rng(258).choice(len(pts),max_points,replace=False)]
    return (pts@transform[:3,:3].T+transform[:3,3]).astype('<f4')


def write_pcd(path, points):
    header=(f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
            f'COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {len(points)}\nDATA binary\n')
    with path.open('wb') as f:
        f.write(header.encode('ascii'));f.write(points.tobytes(order='C'))


def save_sample(session, index, points, T, q, timing, roi, depth, color):
    # A committed sample has its JSON manifest; the solver validates all files.
    stem=f'view{index}'
    write_pcd(session/f'{stem}d.pcd',points)
    if not cv2.imwrite(str(session/f'{stem}_depth.png'),depth):
        raise RuntimeError('Failed to save raw depth image')
    if not cv2.imwrite(str(session/f'{stem}_color.png'),color):
        raise RuntimeError('Failed to save color image')
    write_json(session/f'{stem}.json',dict(index=index,q_rad=q.tolist(),T_base_gripper=T.tolist(),
        roi_depth_xywh=list(map(int,roi)),points=len(points),timing=timing,
        point_frame='handeye_color_optical',point_unit='m',pose_unit='m'))
    rows=[]
    for i in range(1,index+1):
        rec=json.loads((session/f'view{i}.json').read_text())
        pose=np.array(rec['T_base_gripper']);rpy=Rotation.from_matrix(pose[:3,:3]).as_euler('xyz')
        rows.append(','.join(f'{v:.12g}' for v in [*rpy,*(pose[:3,3]*1000)]))
    tmp=session/'RobotPoses.dat.tmp';tmp.write_text('\n'.join(rows)+'\n');tmp.replace(session/'RobotPoses.dat')


def start_camera(serial):
    ctx=rs.context();devices=list(ctx.query_devices())
    if not devices:raise ValueError('未检测到 RealSense，相机未连接')
    if not serial and len(devices)>1:raise ValueError('多个 RealSense，请用 --serial 指定 D405')
    dev=next((d for d in devices if not serial or d.get_info(rs.camera_info.serial_number)==serial),None)
    if dev is None:raise ValueError('未找到指定相机序列号')
    serial=dev.get_info(rs.camera_info.serial_number)
    modes={rs.stream.depth:set(),rs.stream.color:set()}
    for sensor in dev.query_sensors():
        for profile in sensor.get_stream_profiles():
            if not profile.is_video_stream_profile():continue
            stream=profile.stream_type()
            if stream not in modes or profile.fps()<5 or profile.fps()>30:continue
            if stream==rs.stream.depth and profile.format()!=rs.format.z16:continue
            if stream==rs.stream.color and profile.format()!=rs.format.rgb8:continue
            vp=profile.as_video_stream_profile()
            modes[stream].add((vp.width(),vp.height(),profile.fps()))
    pairs=[(d,c) for d in modes[rs.stream.depth] for c in modes[rs.stream.color]
           if d[2]==c[2] and d[0]<=1280 and c[0]<=1920]
    # Prefer 640x480 depth, then the best shared frame rate and color resolution.
    pairs.sort(key=lambda p:(p[0][:2]==(640,480),p[0][2],p[0][0]*p[0][1],p[1][0]*p[1][1]),reverse=True)
    pipe=rs.pipeline();wrapper=rs.pipeline_wrapper(pipe);errors=[]
    for d,c in pairs:
        cfg=rs.config();cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth,*d[:2],rs.format.z16,d[2])
        cfg.enable_stream(rs.stream.color,*c[:2],rs.format.rgb8,c[2])
        try:
            if not cfg.can_resolve(wrapper):continue
            profile=pipe.start(cfg)
            return pipe,profile,dev.get_info(rs.camera_info.name),d,c
        except RuntimeError as exc:
            errors.append(str(exc))
    raise RuntimeError('相机没有可用的深度+彩色组合: '+('; '.join(errors[-2:]) or '无兼容模式'))


def capture(args):
    lock_path=Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))/f'nero-handeye-{os.getuid()}.lock'
    with lock_path.open('w') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('请先关闭已有手眼/相机窗口，它正在占用相机')
        session=ROOT/'calibration_data'/('icp_'+datetime.now().strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}')
        session.mkdir(parents=True)
        pipe=None;bus=None
        try:
            # Snapshot the candidate to keep each acquisition reproducible.
            write_json(session/'initial_calibration.json',json.loads(INITIAL.read_text()))
            bus=PassiveJoints(args.can);fk=FK(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf')
            pipe,profile,name,dm,cm=start_camera(args.serial)
            dprofile=profile.get_stream(rs.stream.depth).as_video_stream_profile()
            cprofile=profile.get_stream(rs.stream.color).as_video_stream_profile()
            Tcd=depth_to_color(dprofile.get_extrinsics_to(cprofile))
            scale=profile.get_device().first_depth_sensor().get_depth_scale()
            intr=dprofile.get_intrinsics()
            write_json(session/'session.json',dict(can=args.can,camera=name,serial=profile.get_device().get_info(rs.camera_info.serial_number),
                depth_mode=dm,color_mode=cm,depth_scale_m=scale,T_color_depth=Tcd.tolist(),
                depth_intrinsics=dict(width=intr.width,height=intr.height,fx=intr.fx,fy=intr.fy,ppx=intr.ppx,ppy=intr.ppy,
                                      model=str(intr.model),coeffs=list(intr.coeffs)),
                cloud_frame='handeye_color_optical',pose_frame='base_link->gripper_flange',
                initial_calibration='initial_calibration.json',pairing='host receipt; stationary window; not hardware synchronized',
                no_motor_commands=True))
            print(f'保存目录: {session}\n深度 {dm}; 彩色 {cm}\n按 s 采样并框选物体，空格/回车确认；按 q 结束。',flush=True)
            print('采样时先停稳，框选范围尽量只包括固定物体；每帧都需要框选。',flush=True)
            cloud=rs.pointcloud();index=0
            while True:
                frames=pipe.wait_for_frames(timeout_ms=4000);host_time=time.monotonic()
                depth=frames.get_depth_frame();color=frames.get_color_frame()
                if not depth or not color:continue
                raw=np.asanyarray(depth.get_data()).copy()
                bgr=cv2.cvtColor(np.asanyarray(color.get_data()),cv2.COLOR_RGB2BGR)
                depth_m=raw.astype(np.float32)*scale
                depth_shown=np.clip((depth_m-args.z_min)/(args.z_max-args.z_min)*255,0,255).astype(np.uint8)
                display=cv2.applyColorMap(depth_shown,cv2.COLORMAP_JET)
                display[raw==0]=0
                cv2.putText(display,f'Samples: {index} | S capture | Q quit',(8,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
                cv2.imshow('ICP depth: S capture, Q finish',display)
                cv2.imshow('ICP color reference',bgr)
                key=cv2.waitKey(1)&255
                if key in (ord('q'),27):break
                if key!=ord('s'):continue
                try:
                    q,timing=bus.snapshot(host_time)
                    verts=np.asanyarray(cloud.calculate(depth).get_vertices()).view(np.float32).reshape(raw.shape[0],raw.shape[1],3).copy()
                    print('请在深度画面框选物体，按空格或回车确认；Esc 取消。',flush=True)
                    roi=cv2.selectROI('Select object, ENTER confirm',display,showCrosshair=True,fromCenter=False)
                    cv2.destroyWindow('Select object, ENTER confirm')
                    if not roi[2] or not roi[3]:print('已取消本次采样',flush=True);continue
                    current,_=bus.snapshot(time.monotonic())
                    if np.max(np.abs(current-q))>np.deg2rad(.3):raise ValueError('框选期间机械臂变化，请重新停稳采样')
                    points=select_points(verts,roi,Tcd,args.z_min,args.z_max)
                    save_sample(session,index+1,points,fk(q),q,timing,roi,raw,bgr)
                    index+=1;print(f'已保存第 {index} 组，{len(points)} 点。',flush=True)
                except (ValueError,RuntimeError) as exc:print(f'未保存：{exc}',flush=True)
            print(f'采样结束：{index} 组，目录：{session}',flush=True)
            if index>=5:
                print('解算命令（复制运行）：',
                      'bash scripts/run_icp_handeye.sh solve '+shlex.quote(str(session)),flush=True)
            else:
                print('少于 5 组，尚不能解算。',flush=True)
        finally:
            cv2.destroyAllWindows()
            if pipe is not None:
                try:pipe.stop()
                except RuntimeError:pass
            if bus is not None:bus.close()


def solve(args):
    session=args.session.resolve();meta=json.loads((session/'session.json').read_text())
    if meta.get('cloud_frame')!='handeye_color_optical':raise ValueError('Point cloud frame mismatch')
    files=sorted(session.glob('view[0-9]*.json'),key=lambda p:int(p.stem[4:]))
    if len(files)<5:raise ValueError(f'需至少 5 组采样，目前 {len(files)} 组')
    for n,f in enumerate(files,1):
        if f.name!=f'view{n}.json' or not (session/f'view{n}d.pcd').is_file():
            raise ValueError('采样编号不连续或点云丢失')
    if not (session/'RobotPoses.dat').exists():raise ValueError('RobotPoses.dat 不存在')
    if len((session/'RobotPoses.dat').read_text().splitlines())!=len(files):raise ValueError('位姿数量和点云不一致')
    if not EXE.exists():raise ValueError('RegHEC 未编译，请先运行 scripts/build_icp_handeye.sh')
    initial_file=args.initial if args.initial is not None else session/'initial_calibration.json'
    candidate=json.loads(initial_file.read_text());x=np.array(candidate['T_gripper_camera'],float)
    if args.initial is None and (meta['serial']!='262622274345' or meta['can']!='can1'):
        raise ValueError('默认初值属于 can1 的 D405；其他相机/机械臂请通过 --initial 指定对应初值')
    if x.shape!=(4,4) or not np.isfinite(x).all() or not np.allclose(x[3],[0,0,0,1]) or not np.allclose(x[:3,:3].T@x[:3,:3],np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(x[:3,:3]),1,atol=1e-5):
        raise ValueError('初始手眼矩阵无效')
    clouds=[];poses=[]
    for f in files:
        rec=json.loads(f.read_text());raw=(session/f"{f.stem}d.pcd").read_bytes().split(b'DATA binary\n',1)
        if len(raw)!=2:raise ValueError(f'非本程序生成的二进制 PCD：{f.stem}')
        points=np.frombuffer(raw[1],dtype='<f4').reshape(-1,3)
        if len(points)!=rec['points'] or len(points)<300 or not np.isfinite(points).all():raise ValueError(f'点云损坏：{f.stem}')
        if len(points)>5000:points=points[np.random.default_rng(258).choice(len(points),5000,replace=False)]
        clouds.append(points.astype(float));poses.append(np.array(rec['T_base_gripper'],float))

    def registration_metrics(transform):
        transformed=[pts@(pose@transform)[:3,:3].T+(pose@transform)[:3,3] for pts,pose in zip(clouds,poses)]
        residual=[]
        for a,b in zip(transformed,transformed[1:]):
            if len(a)>len(b):a,b=b,a
            d,_=cKDTree(b).query(a);residual.extend(d.tolist())
        residual=np.sort(np.asarray(residual))
        trimmed=residual[:max(1,int(len(residual)*.9))]
        return dict(trimmed_nearest_rms_mm=float(np.sqrt(np.mean(trimmed**2))*1000),
                    fraction_within_5mm=float(np.mean(residual<.005)),pair_count=len(clouds)-1)
    fit_before=registration_metrics(x)
    initial=session/'initial_matrix.txt';np.savetxt(initial,x,fmt='%.15g')
    env=os.environ.copy();env['LD_LIBRARY_PATH']=str(LIB)+os.pathsep+env.get('LD_LIBRARY_PATH','')
    print(f'用 {len(files)} 组点云、机械臂姿态和候选外参离线执行 AA-ICP。',flush=True)
    proc=subprocess.run([str(EXE),str(session),str(initial)],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=300)
    (session/'solve.log').write_text(proc.stdout)
    print(proc.stdout,flush=True)
    if proc.returncode:raise RuntimeError(f'算法退出状态 {proc.returncode}，请查看 solve.log')
    result=np.loadtxt(session/'handeye_matrix.csv',delimiter=',')
    if result.shape!=(4,4) or not np.isfinite(result).all() or not np.allclose(result[3],[0,0,0,1],atol=1e-6) or not np.allclose(result[:3,:3].T@result[:3,:3],np.eye(3),atol=1e-4) or not np.isclose(np.linalg.det(result[:3,:3]),1,atol=1e-4):raise ValueError('ICP 结果不是有效位姿')
    delta=np.linalg.norm(result[:3,3]-x[:3,3])*1000
    angle=np.rad2deg(Rotation.from_matrix(x[:3,:3].T@result[:3,:3]).magnitude())
    write_json(session/'icp_candidate.json',dict(T_gripper_camera=result.tolist(),initial_T_gripper_camera=x.tolist(),
        initial_file=str(initial_file),can=meta['can'],camera_serial=meta['serial'],point_frame=meta['cloud_frame'],
        position_change_mm=float(delta),rotation_change_deg=float(angle),samples=len(files),
        same_session_fit_before=fit_before,same_session_fit_after=registration_metrics(result),
        status='candidate_pending_independent_validation',no_motor_commands=True,
        note='ICP fit is not independent physical accuracy; compare held-out observations before use.'))
    print(f'候选已保存 {session}/icp_candidate.json；相比初值平移变化 {delta:.2f}mm，旋转变化 {angle:.2f}°。未替换现有标定。')
    fit_after=registration_metrics(result)
    print('同批相邻点云最近点 RMS（截取90%，非独立精度）：'
          f"{fit_before['trimmed_nearest_rms_mm']:.2f} → {fit_after['trimmed_nearest_rms_mm']:.2f} mm；"
          f"5mm内点占比 {fit_before['fraction_within_5mm']:.0%} → {fit_after['fraction_within_5mm']:.0%}。")


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    c=sub.add_parser('collect');c.add_argument('--can',required=True);c.add_argument('--serial',default='262622274345')
    c.add_argument('--z-min',type=float,default=.07);c.add_argument('--z-max',type=float,default=.60)
    s=sub.add_parser('solve');s.add_argument('session',type=Path);s.add_argument('--initial',type=Path,default=None)
    args=p.parse_args()
    if args.action=='collect' and not (0<args.z_min<args.z_max):p.error('depth range invalid')
    try:(capture if args.action=='collect' else solve)(args)
    except KeyboardInterrupt:print('\n已停止。')
    except (ValueError,RuntimeError,OSError,subprocess.TimeoutExpired) as exc:raise SystemExit(f'错误：{exc}')

if __name__=='__main__':main()
