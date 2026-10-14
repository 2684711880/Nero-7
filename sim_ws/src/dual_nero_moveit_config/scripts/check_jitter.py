#!/usr/bin/env python3
"""Read-only idle stability check. Run while the robot is holding a stationary goal."""
import json,time,math
import rclpy
from sensor_msgs.msg import JointState
rclpy.init();n=rclpy.create_node('measure_dual_idle');rows=[]
s=n.create_subscription(JointState,'/joint_states',lambda m:rows.append(dict(zip(m.name,m.position))),100)
t=time.monotonic()
while time.monotonic()-t<6:rclpy.spin_once(n,timeout_sec=.1)
if not rows:raise RuntimeError('No joint feedback')
ranges={k:max(r[k] for r in rows)-min(r[k] for r in rows) for k in rows[0]}
arms={k:v for k,v in ranges.items() if '_gripper_' not in k};grippers={k:v for k,v in ranges.items() if '_gripper_' in k}
report={'samples':len(rows),'idle_peak_to_peak':ranges,'max_arm_rad':max(arms.values()),'max_gripper_m':max(grippers.values())}
print(json.dumps(report,indent=2))
passed=all(math.isfinite(v) for r in rows for v in r.values()) and report['max_arm_rad']<0.005 and report['max_gripper_m']<0.001
n.destroy_node();rclpy.shutdown();raise SystemExit(0 if passed else 1)
