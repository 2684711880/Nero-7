#!/usr/bin/env python3
"""Feedback-only other arm for dual-arm display/collision checking; no SDK/CAN writes."""
import argparse,time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from handeye_charuco import PassiveJoints


def main():
    p=argparse.ArgumentParser();p.add_argument('--can',required=True);a,ros=p.parse_known_args()
    rclpy.init(args=ros);node=Node('pointing_passive_right');bus=PassiveJoints(a.can)
    pub=node.create_publisher(JointState,'/right/feedback/joint_states',10)
    def tick():
        with bus.lock:history=list(bus.history)
        if not history or time.monotonic()-history[-1][0]>.1:return
        m=JointState();m.header.stamp=node.get_clock().now().to_msg()
        m.name=[f'joint{i}' for i in range(1,8)];m.position=history[-1][2].tolist();pub.publish(m)
    node.create_timer(.02,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        bus.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
