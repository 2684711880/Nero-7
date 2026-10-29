#!/usr/bin/env python3
"""Feedback-only other arm for dual-arm display/collision checking; no SDK/CAN writes."""
import argparse,time,socket,struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from handeye_charuco import PassiveJoints


def main():
    p=argparse.ArgumentParser();p.add_argument('--can',required=True);p.add_argument('--side',choices=['left','right'],default='right');a,ros=p.parse_known_args()
    rclpy.init(args=ros);node=Node('pointing_passive_'+a.side);bus=PassiveJoints(a.can)
    pub=node.create_publisher(JointState,f'/{a.side}/feedback/joint_states',10)
    gripper=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
    gripper.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,struct.pack('=II',0x2A8,0x7FF))
    gripper.bind((a.can,));gripper.setblocking(False)
    last_gripper=None
    def tick():
        nonlocal last_gripper
        while True:
            try: frame=gripper.recv(16)
            except BlockingIOError: break
            cid,size,data=struct.unpack('=IB3x8s',frame)
            if cid==0x2A8 and size==8:
                width,force,status,mode=struct.unpack('>ihBB',data)
                if mode==0:last_gripper=(time.monotonic(),width/1e6)
        with bus.lock:history=list(bus.history)
        if not history or time.monotonic()-history[-1][0]>.1:return
        m=JointState();m.header.stamp=node.get_clock().now().to_msg()
        m.name=[f'joint{i}' for i in range(1,8)];m.position=history[-1][2].tolist()
        if last_gripper and time.monotonic()-last_gripper[0]<.1:
            m.name.append('gripper');m.position.append(last_gripper[1])
        pub.publish(m)
    node.create_timer(.02,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        gripper.close();bus.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
