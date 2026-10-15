#!/usr/bin/env python3
"""C++ official-shaped API -> ROS -> upstream Python callbacks -> mock SDK; no CAN."""
import json
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace
from unittest.mock import Mock
import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped,PoseArray
from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


def main():
    rclpy.init();node=rclpy.create_node('verify_arm_api')
    sdk=Mock();sdk.get_tcp2flange_pose.side_effect=lambda p:p
    stub=SimpleNamespace(agx_arm=sdk,_check_can_control=lambda:True,
        arm_joint_names=['joint'+str(i) for i in range(1,8)],
        _safe_get_value=lambda array,index,default=0.0:AgxArmRosNode._safe_get_value(None,array,index,default),
        _control_gripper_joint=lambda *_:None,_control_hand_joints=lambda *_:None)
    stub._create_pose_cmd=lambda p:AgxArmRosNode._create_pose_cmd(stub,p)
    counts={name:0 for name in ('move_j','move_p','move_l','move_c')}
    def callback(name,msg):
        getattr(AgxArmRosNode,'_'+name+'_callback')(stub,msg)
        counts[name]+=1
    subscriptions=[]
    for name,kind in [('move_j',JointState),('move_p',PoseStamped),('move_l',PoseStamped),('move_c',PoseArray)]:
        subscriptions.append(node.create_subscription(kind,'/left/control/'+name,
            lambda msg,key=name:callback(key,msg),1))
    p=subprocess.Popen(['ros2','run','nero_official_commander_cpp','api_smoke','--ros-args',
        '-p','mode:=mock','-p','send:=true'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    deadline=time.monotonic()+20
    try:
        while p.poll() is None:
            if time.monotonic()>deadline:p.kill();raise RuntimeError('C++ API timed out')
            rclpy.spin_once(node,timeout_sec=.02)
        for _ in range(20):rclpy.spin_once(node,timeout_sec=.01)
        output=p.communicate()[0]
        assert p.returncode==0,output
        assert all(v==1 for v in counts.values()),counts
        sdk.move_j.assert_called_once_with([.1,.2,.3,.4,.5,.6,.7])
        sdk.move_p.assert_called_once_with([-.4,-.2,.3,0,0,0])
        sdk.move_l.assert_called_once_with([-.4,.2,.3,0,0,0])
        sdk.move_c.assert_called_once_with([-.4,-.2,.3,0,0,0],[-.4,0,.4,0,0,0],[-.4,.2,.3,0,0,0])
        print(json.dumps({'passed':True,'upstream_callback_counts':counts,'hardware_connected':False},indent=2))
    finally:
        if p.poll() is None:p.kill();p.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
