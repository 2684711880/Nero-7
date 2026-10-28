"""Integration check: ONLY domain 93 with driver-free mock_stack + mock_arms."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Empty

assert os.environ.get('ROS_DOMAIN_ID') == '93', 'Run only against the isolated CAN-free fixture'
circle = '--circle' in sys.argv
ptp = '--ptp' in sys.argv
sequence = '--sequence' in sys.argv
rclpy.init()
node = rclpy.create_node('verify_straight_line_no_can')
commands = {'left': [], 'right': []}
displays = []
feedback = {}
refresh = {'start': 0, 'goal': 0}
for kind in refresh:
    def updated(msg, key=kind):
        refresh[key] += 1
    node.create_subscription(Empty, f'/rviz/moveit/update_{kind}_state', updated, 10)
for side in commands:
    node.create_subscription(JointState, f'/{side}/control/joint_states',
                             lambda msg, s=side: commands[s].append(msg), 10)
node.create_subscription(DisplayTrajectory, '/display_planned_path', displays.append, 10)
node.create_subscription(JointState, '/joint_states',
                         lambda msg: feedback.update(zip(msg.name, msg.position)), 10)


def invoke(execute):
    cmd = ['bash', '/home/a/Nero-7/scripts/point_to_point.sh' if ptp else '/home/a/Nero-7/scripts/straight_line.sh']
    if sequence:
        cmd = ['ros2', 'run', 'nero_official_commander_cpp', 'task_sequence']
    if circle:
        cmd += ['--ros-args', '-p', 'shape:=circle', '-p', 'arm:=right', '-p', 'radius:=0.03']
    # Both modes use the normal command: only the exact confirmation word executes.
    child = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True)
    child.stdin.write(('do\n'*3 if sequence else 'do\n') if execute else '\n')
    child.stdin.close()
    deadline = time.monotonic()+35
    while child.poll() is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    if child.poll() is None:
        child.terminate()
        raise RuntimeError('straight_line timed out')
    output = child.stdout.read()
    label = 'execution' if execute else 'preview'
    if circle:
        label = 'circle_' + label
    if ptp:
        label = 'ptp_' + label
    if sequence:
        label = 'sequence_' + label
    Path(f'/home/a/Nero-7/hardware_ws/validation/dual_moveit/straight_line_{label}.log').write_text(output)
    assert child.returncode == 0, output
    return output


for _ in range(30):
    rclpy.spin_once(node, timeout_sec=.02)
preview_output = invoke(False)
assert displays, 'No RViz preview'
if sequence:
    # MoveIt itself can also publish the same PTP display, so count task segments, not DDS messages.
    assert preview_output.count('PREVIEW:') == 1, 'Cancellation must stop the task before later segments'
    assert 'remaining actions skipped' in preview_output
assert not any(commands.values()), 'Preview sent motion commands'
assert refresh == {'start': 0, 'goal': 0}, 'Preview must not overwrite a manually chosen RViz target'
display = displays[-1]
preview_count = len(display.trajectory[0].joint_trajectory.points)
output = invoke(True)
assert 'feedback confirms endpoint' in output, output
assert (commands['right'] and not commands['left']) if (circle or ptp or sequence) else (commands['left'] and not commands['right']), 'Incorrect arm routing'
expected_refresh = 3 if sequence else 1
assert refresh == {'start': expected_refresh, 'goal': expected_refresh}, refresh
if sequence:
    assert output.count('PREVIEW:') == 3, 'Expected three confirmed segment previews'
    assert output.count('feedback confirms endpoint') == 3
trajectory = displays[-1].trajectory[0].joint_trajectory
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=.02)
max_error = max(abs(feedback[name]-goal) for name, goal in
                zip(trajectory.joint_names, trajectory.points[-1].positions))
assert max_error < .017454
report = {'hardware_connected': False, 'preview_motion_commands': 0,
          'rviz_start_goal_refresh_events': refresh,
          'preview_points': preview_count, 'left_command_frames': len(commands['left']),
          'right_command_frames': len(commands['right']),
          'feedback_endpoint_max_error_rad': max_error,
          'duration_s': trajectory.points[-1].time_from_start.sec +
                        trajectory.points[-1].time_from_start.nanosec*1e-9}
if circle:
    from moveit_msgs.srv import GetPositionFK
    from copy import deepcopy
    fk_client = node.create_client(GetPositionFK, '/compute_fk')
    assert fk_client.wait_for_service(timeout_sec=5)
    def fk(state):
        request = GetPositionFK.Request()
        request.header.frame_id = 'world'
        request.fk_link_names = ['right_tcp_link']
        request.robot_state = state
        future = fk_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.result().error_code.val == 1
        p = future.result().pose_stamped[0].pose.position
        return (p.x, p.y, p.z)
    import math
    start = displays[-1].trajectory_start
    a = fk(start)
    positions = []
    for point in trajectory.points:
        state = deepcopy(start)
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        values.update(zip(trajectory.joint_names, point.positions))
        state.joint_state.name = list(values)
        state.joint_state.position = list(values.values())
        positions.append(fk(state))
    report['circle_radius_m'] = .03
    report['circle_radial_max_error_m'] = max(abs(math.hypot(p[0]-(a[0]+.03),p[1]-a[1])-.03) for p in positions)
    report['circle_plane_max_error_m'] = max(abs(p[2]-a[2]) for p in positions)
    report['circle_closure_error_m'] = math.dist(positions[-1],a)
    assert report['circle_radial_max_error_m'] < .001
    assert report['circle_plane_max_error_m'] < .001
    assert report['circle_closure_error_m'] < .001
report_name = 'circle_report.json' if circle else 'straight_line_report.json'
if sequence:
    report_name = 'task_sequence_report.json'
    report['confirmed_motion_segments'] = 3
    report['cancelled_task_previews'] = 1
if ptp:
    report_name = 'point_to_point_report.json'
    from moveit_msgs.srv import GetPositionFK
    from moveit_msgs.msg import RobotState
    import math
    request = GetPositionFK.Request()
    request.header.frame_id = 'world'
    request.fk_link_names = ['right_tcp_link']
    request.robot_state = displays[-1].trajectory_start
    values = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position))
    values.update(feedback)
    request.robot_state.joint_state.name = list(values)
    request.robot_state.joint_state.position = list(values.values())
    client = node.create_client(GetPositionFK, '/compute_fk')
    assert client.wait_for_service(timeout_sec=5)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    assert future.result().error_code.val == 1
    p = future.result().pose_stamped[0].pose.position
    report['target_xyz'] = [.20, -.40, .60]
    report['feedback_tcp_xyz'] = [p.x, p.y, p.z]
    report['feedback_tcp_target_error_m'] = math.dist([p.x, p.y, p.z], report['target_xyz'])
    assert report['feedback_tcp_target_error_m'] < .002
Path('/home/a/Nero-7/hardware_ws/validation/dual_moveit', report_name).write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
node.destroy_node()
rclpy.shutdown()
