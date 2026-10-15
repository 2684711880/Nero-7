"""Independent simulation-only repetition after the complete task passes."""
import json
import math
import subprocess
import time
from pathlib import Path
folder = Path(__file__).parent
end = time.monotonic()+350
while not (folder/'report.json').exists():
    if time.monotonic() > end:
        raise TimeoutError('Main-task verification did not finish')
    time.sleep(.5)
assert json.loads((folder/'report.json').read_text())['passed'], 'Main task did not pass'
task = {'version':1, 'task_id':'repeat_turns_004', 'left_arm_actions':[], 'right_arm_actions':[],
        'base_actions':[{'action_id':'turn_'+str(i), 'type':'base_move_to',
            'parameters':{'frame_id':'odom', 'position_xy_m':[0,0], 'yaw_rad':angle},
            'wait_for':[], 'timeout_s':60} for i,angle in enumerate([math.pi,0],1)]}
path = folder/'repeat_turns.json'
path.write_text(json.dumps(task, indent=2)+'\n')
with (folder/'repeat_turns.log').open('w') as out:
    result = subprocess.run(['ros2','run','nero_task_executor','submit_task',str(path),'--timeout','150'],
                            stdout=out, stderr=subprocess.STDOUT, timeout=160)
state = json.loads((folder/'state.json').read_text())['records'][task['task_id']]['state']
report = {'passed':result.returncode==0 and state['status']=='SUCCEEDED',
          'task_status':state, 'turns':2, 'hardware_connected':False}
(folder/'repeat_turns_report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({'passed':report['passed'], 'turns':2, 'status':state['status']}))
raise SystemExit(0 if report['passed'] else 1)
