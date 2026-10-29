"""Exercise the overwrite bug with competing publishers, without hardware."""
import os
import sys
import time
import unittest
from pathlib import Path

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_hole_preview import PREVIEW_TOPIC, preview_publisher


class IsolationTest(unittest.TestCase):
    def test_shared_old_messages_cannot_overwrite_task_preview(self):
        self.assertNotEqual(PREVIEW_TOPIC,'/display_planned_path')
        rclpy.init()
        node=Node('preview_isolation_test')
        received=[];shared=[]
        task=preview_publisher(node)
        old=node.create_publisher(DisplayTrajectory,'/display_planned_path',10)
        task_sub=node.create_subscription(DisplayTrajectory,PREVIEW_TOPIC,lambda m:received.append(m.model_id),10)
        old_sub=node.create_subscription(DisplayTrajectory,'/display_planned_path',lambda m:shared.append(m.model_id),10)
        try:
            deadline=time.monotonic()+3
            while (task.get_subscription_count()==0 or old.get_subscription_count()==0) and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
            self.assertGreater(task.get_subscription_count(),0)
            self.assertGreater(old.get_subscription_count(),0)
            task.publish(DisplayTrajectory(model_id='reviewed_current'))
            for _ in range(20):
                old.publish(DisplayTrajectory(model_id='unrelated_old_park'))
                rclpy.spin_once(node,timeout_sec=.02)
            deadline=time.monotonic()+.5
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
            self.assertTrue(shared,'The competing old publisher must actually have delivered messages')
            self.assertEqual(received,['reviewed_current'])
        finally:
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
