"""No-CAN integration: existing MoveIt/controller/bridge chain, simulated bottle contact.
Run with sourced hardware workspace and ROS_DOMAIN_ID=97.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from moveit_msgs.msg import DisplayTrajectory
from rcl_interfaces.srv import GetParameters

assert os.environ.get('ROS_DOMAIN_ID') == '97', 'Use isolated domain 97'
root = Path('/home/a/Nero-7')
out = root/'hardware_ws/validation/dual_moveit'
out.mkdir(parents=True, exist_ok=True)
stack_log = (out/'gripper_mock_stack.log').open('w')
stack = subprocess.Popen(['ros2','launch',str(root/'hardware_ws/src/nero_dual_bringup/test/mock_stack.launch.py')],stdout=stack_log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init()
node = Node('verify_gripper_no_can')
names = [f'joint{i}' for i in range(1,8)]+['gripper']
initial = [math.radians(v) for v in (11.443,12.308,6.283,81.054,-.298,17.648,-3.459)]+[.04]
states = {s:dict(zip(names,initial)) for s in ('left','right')}
gates = {s:False for s in states}
commands = []
displays = []
pubs = {s:node.create_publisher(JointState,f'/{s}/feedback/joint_states',10) for s in states}
for side in states:
    def gate(request,response,s=side):
        gates[s] = request.data
        response.success = True
        return response
    node.create_service(SetBool,f'/{side}/control_enable',gate)
    def command(msg,s=side):
        commands.append({'side':s,'names':list(msg.name),'positions':list(msg.position),'effort':list(msg.effort)})
        if not gates[s]:
            return
        for name,value in zip(msg.name,msg.position):
            # Bottle prevents complete closure: actual opening stops at 35 mm.
            states[s][name] = max(value,.035) if name=='gripper' else value
    node.create_subscription(JointState,f'/{side}/control/joint_states',command,10)
node.create_subscription(DisplayTrajectory,'/display_planned_path',lambda msg:displays.append(msg),10)
for topic in ('/rviz/moveit/update_start_state','/rviz/moveit/update_goal_state'):
    node.create_subscription(Empty,topic,lambda msg:None,10)
def publish():
    for side,pub in pubs.items():
        msg = JointState(name=names,position=[states[side][n] for n in names])
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
node.create_timer(.01,publish)
thread = threading.Thread(target=rclpy.spin,args=(node,),daemon=True)
thread.start()
report = {'hardware_connected':False,'simulated_bottle_width_m':.035}
try:
    client = node.create_client(GetParameters,'/dual_joint_bridge/get_parameters')
    assert client.wait_for_service(timeout_sec=20), 'bridge unavailable'
    time.sleep(7)
    def run(label, answer, extra=()):
        commands.clear(); displays.clear()
        process = subprocess.Popen(['ros2','run','nero_official_commander_cpp','gripper_sequence',*extra],stdin=subprocess.PIPE,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        if label=='cancel':
            time.sleep(3)  # Let the newly created preview publisher discover the monitor.
        stdout, _ = process.communicate(answer,timeout=45)
        result = subprocess.CompletedProcess(process.args,process.returncode,stdout)
        (out/f'gripper_{label}.log').write_text(result.stdout)
        assert result.returncode==0, result.stdout
        time.sleep(.2)
        report[label] = {'commands':len(commands),'previews':result.stdout.count('PREVIEW:'),'final_width_m':states['right']['gripper']}
        return result.stdout, list(commands)
    log, sent = run('cancel','r\nr\n\n')
    assert not sent and 'Task cancelled' in log
    assert log.count('REPLAY:')==2
    from rclpy.serialization import serialize_message
    encoded = [serialize_message(msg) for msg in displays]
    assert encoded and encoded.count(encoded[-1])>=3, 'Replay must resend identical trajectory'
    report['cancel']['replays']=2
    assert report['cancel']['previews']==1
    log, sent = run('confirm','do\ndo\n')
    assert report['confirm']['previews']==2
    assert 'Gripper opening confirmed' in log and 'Object grasp NOT verified' in log
    assert sent and all(c['side']=='right' and c['names']==['gripper'] for c in sent)
    assert any(c['effort']==[1.7] and abs(c['positions'][0]-.06)<.002 for c in sent)
    assert any(c['effort']==[.8] and c['positions'][0]<.002 for c in sent)
    assert abs(states['right']['gripper']-.035)<1e-6
    log, sent = run('direct','',('--ros-args','-p','preview:=false'))
    assert report['direct']['previews']==0 and 'Object grasp NOT verified' in log
    assert sent and all(c['names']==['gripper'] and c['side']=='right' for c in sent)
    report['passed']=True
    print(json.dumps(report,indent=2))
finally:
    (out/'gripper_report.json').write_text(json.dumps(report,indent=2)+'\n')
    os.killpg(stack.pid,signal.SIGINT)
    try: stack.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(stack.pid,signal.SIGTERM); stack.wait(timeout=5)
    rclpy.shutdown(); thread.join(timeout=2); node.destroy_node(); stack_log.close()
