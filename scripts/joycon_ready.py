#!/usr/bin/env python3
"""Record a taught dual-arm pose passively and restore it using official move_j."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
CHANNELS = {'left': 'can1', 'right': 'can0'}
POSE_FILE = ROOT / 'config/joycon/ready_pose.json'


def official_joint_limits():
    from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD
    return ROBOT_JOINT_LIMIT_PRESET_RAD['nero']


def open_joint_feedback(channel):
    from handeye_charuco import PassiveJoints
    return PassiveJoints(channel)


def valid_pose_document(data):
    return ((data.get('schema') == 1 and data.get('source') == 'passive_can_joint_feedback') or
            (data.get('schema') == 2 and data.get('source') == 'taught_joint_pose'))


def load_ready_degrees():
    if not POSE_FILE.is_file():
        raise RuntimeError(f'尚未记录准备姿态：{POSE_FILE}；请先使用 joycon.sh --record')
    data = json.loads(POSE_FILE.read_text())
    if not valid_pose_document(data):
        raise RuntimeError('准备姿态文件来源或版本无效')
    result = {}
    for side, channel in CHANNELS.items():
        if side not in data.get('arms', {}):
            raise RuntimeError(f'{side}: 尚未记录准备姿态，请摆好后使用 --record --arm {side}')
        arm = data['arms'][side]
        if (side == 'right' and arm.get('source') == 'urdf_mirror_of_left' and
                arm.get('derived_from_left_captured_at') != data['arms']['left'].get('captured_at')):
            raise RuntimeError('左臂记录已更新，请重新执行 --mirror-left-to-right 更新右臂目标')
        q = np.asarray(arm['joints_rad'], dtype=float)
        if arm.get('can') != channel or q.shape != (7,) or not np.isfinite(q).all():
            raise RuntimeError(f'{side}: 准备姿态数据或CAN映射无效')
        result[side] = np.rad2deg(q).tolist()
    return result


def model_report(degrees_by_side=None):
    degrees_by_side = load_ready_degrees() if degrees_by_side is None else degrees_by_side
    root = ET.parse(ROOT / 'sim_ws/src/dual_nero_moveit_config/urdf/dual_nero.urdf').getroot()
    single = ET.parse(ROOT / 'config/gravity/nero_standard_gripper_no_camera.urdf').getroot()
    def origin(joint):
        o = joint.find('origin')
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(o.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        t[:3, 3] = np.fromstring(o.get('xyz', '0 0 0'), sep=' ')
        return t
    report = {}
    limits = official_joint_limits()
    for side, degrees in degrees_by_side.items():
        t = origin(root.find(f"joint[@name='{side}_arm_joint']"))
        positions = []
        for i, q in enumerate(np.deg2rad(degrees), 1):
            joint = root.find(f"joint[@name='{side}_joint{i}']")
            calibrated = single.find(f"joint[@name='joint{i}']")
            if not np.allclose(origin(joint), origin(calibrated), atol=1e-9):
                raise RuntimeError(f'{side} joint{i}: model mismatch')
            lower, upper = limits[f'joint{i}']
            if not lower <= q <= upper:
                raise RuntimeError(f'{side} joint{i}: target outside limits')
            t = t @ origin(joint)
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(np.fromstring(joint.find('axis').get('xyz'), sep=' ') * q).as_matrix()
            t = t @ rotation
            positions.append(t[:3, 3].copy())
        t = t @ origin(root.find(f"joint[@name='{side}_gripper_flange_joint']"))
        upper = (positions[2] - positions[0]) / .31
        forearm = (positions[4] - positions[2]) / .27001
        report[side] = dict(joints_deg=degrees, upper_direction=upper.round(6).tolist(),
                            forearm_direction=forearm.round(6).tolist(),
                            gripper_forward=t[:3, 2].round(6).tolist(),
                            flange_rotation=t[:3, :3].tolist(),
                            flange_xyz=t[:3, 3].round(6).tolist())
    return report


def angles(robot):
    # SDK startup can return a seven-element array with not-yet-received axes
    # still filled with zero. Only accept assembled fresh four-frame snapshots.
    reader = getattr(robot, '_joycon_joint_feedback', None)
    if reader is None:
        raise RuntimeError('完整七轴反馈读取器未初始化')
    if reader.error:
        raise RuntimeError(f'七轴反馈读取失败：{reader.error}')
    with reader.lock:
        sample = reader.history[-1] if reader.history else None
    if sample is None:
        raise RuntimeError('完整七轴反馈缺失')
    oldest, newest, values = sample
    if not (0 <= time.monotonic() - oldest < .1 and 0 <= newest - oldest < .03):
        raise RuntimeError('完整七轴反馈缺失或过期')
    q = np.asarray(values, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise RuntimeError('关节反馈无效')
    return q


def record_pose(sides=('left', 'right')):
    # Reuse the project's receive-only, complete-seven-axis CAN reader.
    from handeye_charuco import PassiveJoints
    from datetime import datetime, timezone
    import shutil
    readers = {}
    try:
        for side in sides:
            readers[side] = PassiveJoints(CHANNELS[side])
        print(f'正在只读采集 {", ".join(sides)}，2.5秒；请保持稳定。不使能、不发送运动命令。', flush=True)
        time.sleep(2.5)
        now = time.monotonic()
        data = dict(schema=2, source='taught_joint_pose',
                    captured_at=datetime.now(timezone.utc).isoformat(), arms={})
        if POSE_FILE.exists():
            previous = json.loads(POSE_FILE.read_text())
            if not valid_pose_document(previous):
                raise RuntimeError('旧准备姿态文件无效，未覆盖')
            data['arms'] = previous['arms'].copy()
        degrees = {}
        for side, reader in readers.items():
            reader.snapshot(now)  # Enforces complete, recent and stationary feedback.
            with reader.lock:
                recent = [sample for sample in reader.history if now - 2 <= sample[0] <= now]
            if len(recent) < 20 or recent[-1][1] - recent[0][0] < 1.5:
                raise RuntimeError(f'{side}: 连续稳定反馈不足1.5秒，未保存')
            samples = np.asarray([sample[2] for sample in recent])
            spread = np.rad2deg(np.ptp(samples, axis=0))
            if np.max(spread) > .5:
                raise RuntimeError(f'{side}: 采集期间关节变化超过0.5°，未保存')
            q = np.median(samples, axis=0)
            degrees[side] = np.rad2deg(q).tolist()
            data['arms'][side] = dict(can=CHANNELS[side], joints_rad=q.tolist(),
                                      source='passive_can_joint_feedback',
                                      captured_at=data['captured_at'],
                                      joints_deg=degrees[side], samples=len(samples),
                                      peak_to_peak_deg=spread.tolist(),
                                      latest_feedback_age_s=now - recent[-1][0])
        data['model_report'] = model_report({side: np.rad2deg(arm['joints_rad']).tolist()
                                            for side, arm in data['arms'].items()})
        POSE_FILE.parent.mkdir(parents=True, exist_ok=True)
        if POSE_FILE.exists():
            backup = POSE_FILE.with_name('ready_pose.' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f') + '.bak.json')
            shutil.copy2(POSE_FILE, backup)
        temporary = POSE_FILE.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
        temporary.replace(POSE_FILE)
        for side, q in degrees.items():
            print(f'{side}: 已记录关节角度°={np.round(q, 3).tolist()}', flush=True)
        print(f'准备姿态已保存：{POSE_FILE}；启动和归位均读取此文件。', flush=True)
        missing = set(CHANNELS) - set(data['arms'])
        if missing:
            print(f'仍待记录：{", ".join(sorted(missing))}；双臂启动和归位暂不可用。', flush=True)
    finally:
        for reader in readers.values():
            reader.close()


def mirror_left_to_right():
    from datetime import datetime, timezone
    import shutil
    if not POSE_FILE.is_file():
        raise RuntimeError('请先记录左臂准备姿态')
    data = json.loads(POSE_FILE.read_text())
    if not valid_pose_document(data) or 'left' not in data.get('arms', {}):
        raise RuntimeError('缺少有效的左臂记录')
    left = data['arms']['left']
    q = np.asarray(left['joints_rad'], dtype=float)
    if left.get('can') != CHANNELS['left'] or q.shape != (7,) or not np.isfinite(q).all():
        raise RuntimeError('左臂记录无效')
    # Derived from corresponding joint axes; includes this model's wrist convention.
    signs = np.array([-1, 1, -1, 1, -1, -1, 1])
    right_q = q * signs
    report = model_report({'left': np.rad2deg(q).tolist(), 'right': np.rad2deg(right_q).tolist()})
    reflection = np.diag([1., -1., 1.])
    position_error = np.linalg.norm(np.asarray(report['right']['flange_xyz']) -
                                    reflection @ np.asarray(report['left']['flange_xyz']))
    expected_rotation = reflection @ np.asarray(report['left']['flange_rotation']) @ reflection
    orientation_error = (Rotation.from_matrix(expected_rotation).inv() *
                         Rotation.from_matrix(report['right']['flange_rotation'])).magnitude()
    if position_error > .0001 or orientation_error > math.radians(.1):
        raise RuntimeError('工程模型的左右法兰镜像校验未通过，未保存')
    stamp = datetime.now(timezone.utc)
    data.update(schema=2, source='taught_joint_pose', captured_at=stamp.isoformat())
    left.setdefault('source', 'passive_can_joint_feedback')
    data['arms']['right'] = dict(can=CHANNELS['right'], joints_rad=right_q.tolist(),
                                joints_deg=np.rad2deg(right_q).tolist(),
                                source='urdf_mirror_of_left', captured_at=stamp.isoformat(),
                                derived_from_left_captured_at=left.get('captured_at', data['captured_at']),
                                mirror_joint_signs=signs.tolist(),
                                model_position_error_m=float(position_error),
                                model_orientation_error_deg=math.degrees(orientation_error),
                                hardware_verified=False)
    data['model_report'] = report
    backup = POSE_FILE.with_name('ready_pose.' + stamp.strftime('%Y%m%dT%H%M%S%f') + '.bak.json')
    shutil.copy2(POSE_FILE, backup)
    temporary = POSE_FILE.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(POSE_FILE)
    print(f'右臂镜像角度°={np.rad2deg(right_q).round(3).tolist()}', flush=True)
    print(f'已保存；法兰镜像位置误差 {position_error*1000:.4f}mm，姿态误差 {math.degrees(orientation_error):.6f}°。未驱动右臂。', flush=True)


def hold(robots):
    for side, robot in robots.items():
        try:
            robot.move_j(angles(robot).tolist())
            print(f'{side}: 已请求当前角度保持。', flush=True)
        except Exception as error:
            print(f'{side}: 请求保持失败：{error}', file=sys.stderr)


def execute(returning=False):
    ready_degrees = load_ready_degrees()
    model_report(ready_degrees)
    from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
    # Avoid taking CAN ownership while another known robot writer is running.
    import subprocess
    processes = subprocess.check_output(['ps', '-eo', 'args='], text=True)
    for line in processes.splitlines():
        if any(token in line for token in ('/agx_arm_ctrl_single', '/move_group',
                                          'joycon_move_p_dual.py --execute', 'nero_zero_force_drag.py')):
            raise RuntimeError('请先停止现有机械臂控制进程：' + line +
                               f'\n请执行 bash {ROOT / "scripts/joycon.sh"} --stop，确认退出后重试。')
    robots = {}
    readers = {}
    motion_sent = False
    try:
        for side, can in CHANNELS.items():
            robot = AgxArmFactory.create_arm(create_agx_arm_config(
                robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=can))
            robots[side] = robot
            readers[side] = open_joint_feedback(can)
            robot._joycon_joint_feedback = readers[side]
            robot.connect()
        deadline = time.monotonic() + 5
        while True:
            try:
                current = {side: angles(robot) for side, robot in robots.items()}
                break
            except RuntimeError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(.02)
        pending = []
        for side, q in current.items():
            print(f'{side}: 当前角度°={np.rad2deg(q).round(2).tolist()}', flush=True)
            limits = official_joint_limits()
            for i, angle in enumerate(q, 1):
                lower, upper = limits[f'joint{i}']
                if not lower <= angle <= upper:
                    raise RuntimeError(f'{side}: joint{i} 当前{math.degrees(angle):.3f}°超出官方限位'
                                       f'[{math.degrees(lower):.3f}°, {math.degrees(upper):.3f}°]，拒绝自动运动')
            if np.max(np.abs(q - np.deg2rad(ready_degrees[side]))) <= math.radians(.5):
                continue
            pending.append(side)
        for side, robot in robots.items():
            robot.set_speed_percent(10)
            deadline = time.monotonic() + 5
            while not robot.enable():
                if time.monotonic() > deadline:
                    raise RuntimeError(f'{side}: 使能未确认')
                time.sleep(.1)
        stages = ('低速归位' if returning else '到记录的准备姿态',)
        for stage in stages:
            targets = {}
            for side in pending:
                target = ready_degrees[side].copy()
                targets[side] = np.deg2rad(target)
                print(f'{side}: {stage}，10%，目标°={target}', flush=True)
                motion_sent = True
                robots[side].move_j(targets[side].tolist())
            deadline = time.monotonic() + 40
            settled_since = None
            while targets:
                settled = True
                for side, target in targets.items():
                    q = angles(robots[side])
                    settled = settled and np.max(np.abs(q - target)) <= math.radians(.5)
                now = time.monotonic()
                if settled:
                    settled_since = now if settled_since is None else settled_since
                    if now - settled_since >= .5:
                        break
                else:
                    settled_since = None
                if now > deadline:
                    raise RuntimeError(f'{stage}: 到位反馈超时')
                time.sleep(.02)
        # Also check an arm which was already at ready and received no command.
        deadline = time.monotonic() + 5
        settled_since = None
        while True:
            settled = all(np.max(np.abs(angles(robot) - np.deg2rad(ready_degrees[side])))
                          <= math.radians(.5) for side, robot in robots.items())
            now = time.monotonic()
            if settled:
                settled_since = now if settled_since is None else settled_since
                if now - settled_since >= .5:
                    break
            else:
                settled_since = None
            if now > deadline:
                raise RuntimeError('双臂最终准备姿态未稳定到位')
            time.sleep(.02)
        print('两臂准备姿态已通过关节反馈确认。' + (
            '归位完成；夹爪开度未改变。' if returning else '接下来启动100%驱动并校准手柄。'), flush=True)
    except BaseException:
        if motion_sent:
            hold(robots)
        raise
    finally:
        for robot in robots.values():
            robot.disconnect()
        for reader in readers.values():
            reader.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true', help='以10%速度执行准备动作；默认仅模型预览')
    action.add_argument('--return', dest='returning', action='store_true',
                        help='退出其他控制进程后，以10%速度从当前关节角直接归位，不依赖手柄')
    action.add_argument('--record', action='store_true', help='只读采集双臂稳定反馈，保存为准备姿态')
    action.add_argument('--mirror-left-to-right', action='store_true', help='根据模型把左臂记录镜像为右臂目标，不驱动机械臂')
    parser.add_argument('--arm', choices=('left', 'right', 'both'), default='both',
                        help='仅用于 --record，指定要记录的机械臂，保留另一侧已有记录')
    args = parser.parse_args()
    if not args.record and args.arm != 'both':
        parser.error('--arm 仅可与 --record 一起使用')
    if args.record:
        record_pose(('left', 'right') if args.arm == 'both' else (args.arm,))
        return
    if args.mirror_left_to_right:
        mirror_left_to_right()
        return
    report = model_report()
    if args.execute or args.returning:
        for side, state in report.items():
            print(f'{side}: 记录目标角度°={np.round(state["joints_deg"], 3).tolist()}', flush=True)
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if args.execute or args.returning:
        execute(returning=args.returning)


if __name__ == '__main__':
    main()
