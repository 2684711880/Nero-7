#!/usr/bin/env python3
"""Exercise the C++ publisher against the running Gazebo command bridge."""
import json
import math
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    root=Path(__file__).resolve().parents[1]
    output=root/'sim_ws/validation/cpp_topic_control'
    output.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=rclpy.create_node('verify_cpp_topic_control')
    buffer=Buffer()
    listener=TransformListener(buffer,node)
    deadline=time.monotonic()+10
    while not buffer.can_transform('odom','left_gripper_base',Time()):
        if time.monotonic()>deadline: raise RuntimeError('No simulation TF')
        rclpy.spin_once(node,timeout_sec=.02)
    results=[]
    def current(side):
        until=time.monotonic()+.2
        while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.01)
        t=buffer.lookup_transform('odom',side+'_gripper_base',Time()).transform.translation
        return [t.x,t.y,t.z]
    def run(side,motion,extra=None,expected=None,success=True):
        start=current(side)
        args=['ros2','run','nero_official_commander_cpp','commander_reference','--ros-args',
              '-r','__ns:=/'+side]
        if motion=='named': args+=['-p','motion:=named','-p','target:=ready']
        else:
            args+=['--params-file',str(root/f'hardware_ws/src/nero_official_commander_cpp/config/sim_left_{motion}.yaml'),
                   '-p',f'tcp_link:={side}_gripper_base']
        args+=extra or []
        args+=['-p','send:=true','-p','wait_result:=true']
        name=side+'_'+motion+('_rejected' if not success else '')
        logfile=output/(name+'.log')
        with logfile.open('w') as log:
            process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,cwd=root)
            deadline=time.monotonic()+90
            while process.poll() is None:
                if time.monotonic()>deadline:
                    process.kill(); process.wait(); raise RuntimeError('Publisher timeout')
                rclpy.spin_once(node,timeout_sec=.02)
        end=current(side)
        text=logfile.read_text()
        passed=(process.returncode==0 and 'SUCCEEDED' in text) if success else (
            process.returncode!=0 and 'Incomplete path; not executing' in text)
        delta=[b-a for a,b in zip(start,end)]
        row={'case':name,'exit_code':process.returncode,'observed_displacement_m':delta,'passed':passed}
        if expected is not None:
            row['endpoint_error_m']=math.dist(delta,expected)
            row['passed']=passed and row['endpoint_error_m']<.002
        results.append(row)
        (output/'report.json').write_text(json.dumps(results,indent=2))
        print(json.dumps(row),flush=True)
        if not row['passed']: raise RuntimeError(f'{name} failed; see {logfile}')
    try:
        for side in ('left','right'):
            run(side,'named')
            run(side,'line',expected=[0,0,-.01])
            extra=['-p','offsets:=[0.0,0.0,0.0,0.0,0.005,0.005,0.0,0.01,0.0]'] if side=='right' else []
            run(side,'arc',extra,expected=[0,.01 if side=='right' else -.01,0])
        run('left','line',['-p','offsets:=[0.0,0.0,2.0]'],expected=[0,0,0],success=False)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
