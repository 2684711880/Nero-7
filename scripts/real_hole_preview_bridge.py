#!/usr/bin/env python3
"""Forward RViz Plan previews only when their start matches current feedback."""
import hashlib
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from real_hole_preview import preview_publisher


def validate_preview(message, values, tolerance=.03):
    if not message.trajectory:
        return False
    start=dict(zip(message.trajectory_start.joint_state.name,message.trajectory_start.joint_state.position))
    names=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
    if any(name not in start or name not in values or not math.isfinite(start[name]) or abs(start[name]-values[name])>tolerance for name in names):
        return False
    first=message.trajectory[0].joint_trajectory
    if not first.points or len(first.joint_names)!=len(first.points[0].positions):
        return False
    if any(name not in values or not math.isfinite(q) or abs(q-values[name])>tolerance for name,q in zip(first.joint_names,first.points[0].positions)):
        return False
    return True


class PreviewBridge(Node):
    def __init__(self):
        super().__init__('real_hole_preview_bridge')
        self.values={};self.stamps={};self.last_synced={};self.last_hash=None
        self.preview=preview_publisher(self)
        self.sync_start=self.create_publisher(Empty,'/rviz/moveit/update_start_state',10)
        self.joints=self.create_subscription(JointState,'/joint_states',self.on_joint,10)
        self.paths=self.create_subscription(DisplayTrajectory,'/display_planned_path',self.on_preview,10)
        self.timer=self.create_timer(.5,self.refresh_gui_start)

    def on_joint(self,message):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if not 0 <= time.time()-stamp < .3:return
        for name,value in zip(message.name,message.position):
            if math.isfinite(value):self.values[name]=value;self.stamps[name]=time.monotonic()

    def fresh(self):
        names=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
        return all(name in self.values and time.monotonic()-self.stamps.get(name,0)<.3 for name in names)

    def refresh_gui_start(self):
        if not self.fresh() or self.sync_start.get_subscription_count()==0:return
        if not self.last_synced or any(abs(self.values[name]-value)>.005 for name,value in self.last_synced.items()):
            self.sync_start.publish(Empty())
            self.last_synced={name:value for name,value in self.values.items() if '_joint' in name and 'gripper' not in name}

    def on_preview(self,message):
        if not self.fresh() or not validate_preview(message,self.values):
            self.get_logger().warning('忽略共享话题中起点过期或不完整的预览',throttle_duration_sec=5.)
            return
        digest=hashlib.sha256(serialize_message(message)).digest()
        if digest==self.last_hash:return
        self.last_hash=digest
        self.preview.publish(message)
        self.get_logger().info('RViz Plan 预览已校验当前起点并转发')


def main():
    rclpy.init();node=PreviewBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
