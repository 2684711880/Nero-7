#!/usr/bin/env python3
"""Run the C++ circle example against Gazebo, check measured TCP geometry."""
import json
import math
from pathlib import Path
import re
import subprocess
import time
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener


def main():
    root=Path(__file__).resolve().parents[1]
    output=root/'sim_ws/validation/xy_circle';output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('verify_xy_circle')
    buffer=Buffer();listener=TransformListener(buffer,node)
    states=[];samples=[];record=[False]
    def result(message):
        states.append(message.data)
        if message.data.startswith('SUCCEEDED joints'):record[0]=True
    subscription=node.create_subscription(String,'/left/control/result',result,10)
    deadline=time.monotonic()+45
    while not buffer.can_transform('base_link','left_gripper_base',Time()) or node.count_publishers('/left/control/result')==0:
        if time.monotonic()>deadline:raise RuntimeError('Simulation not ready')
        rclpy.spin_once(node,timeout_sec=.02)
    def sample():
        if not record[0]:return
        tf=buffer.lookup_transform('base_link','left_gripper_base',Time())
        stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
        if samples and stamp<=samples[-1][0]:return
        t=tf.transform.translation;q=tf.transform.rotation
        samples.append([stamp,t.x,t.y,t.z,q.x,q.y,q.z,q.w])
    with (output/'commander.log').open('w') as log:
        process=subprocess.Popen(['bash',str(root/'scripts/sim_commander.sh'),'--ros-args','-p','send:=true'],
            cwd=root,stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+90
        try:
            while process.poll() is None:
                if time.monotonic()>deadline:process.kill();raise RuntimeError('C++ execution timed out')
                rclpy.spin_once(node,timeout_sec=.01);sample()
            for _ in range(20):rclpy.spin_once(node,timeout_sec=.01);sample()
        finally:
            if process.poll() is None:process.kill()
            process.wait()
    text=(output/'commander.log').read_text()
    matched=re.search(r'centre=\(([^)]+)\) m, radius=([0-9.]+)',text)
    report={'exit_code':process.returncode,'states':states,'samples':len(samples),'passed':False}
    if matched and samples:
        centre=[float(v) for v in matched[1].split(',')];radius=float(matched[2])
        radial=max(abs(math.hypot(p[1]-centre[0],p[2]-centre[1])-radius) for p in samples)
        plane=max(abs(p[3]-centre[2]) for p in samples)
        closure=math.dist(samples[0][1:4],samples[-1][1:4])
        travel=sum(math.dist(a[1:4],b[1:4]) for a,b in zip(samples,samples[1:]))
        report.update(centre_m=centre,radius_m=radius,maximum_radial_error_m=radial,
            maximum_plane_error_m=plane,closure_error_m=closure,observed_travel_m=travel)
        report['passed']=(process.returncode==0 and text.count('SUCCEEDED arc')==2 and
            radial<.002 and plane<.002 and closure<.002 and travel>.8*2*math.pi*radius)
    (output/'tf_samples.json').write_text(json.dumps(samples))
    (output/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()
    if not report['passed']:raise RuntimeError('Circle verification failed; inspect commander.log')


if __name__=='__main__':main()
