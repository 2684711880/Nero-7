import copy
import sys
import time
import unittest
from pathlib import Path

import rclpy
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Empty

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_hole_preview import PREVIEW_TOPIC
from real_hole_preview_bridge import PreviewBridge


class BridgeTest(unittest.TestCase):
    def test_gui_current_plan_visible_but_later_old_park_not_visible(self):
        rclpy.init();node=PreviewBridge();received=[];sync=[]
        sink=node.create_subscription(DisplayTrajectory,PREVIEW_TOPIC,lambda m:received.append(m),10)
        start_sink=node.create_subscription(Empty,'/rviz/moveit/update_start_state',lambda m:sync.append(m),10)
        joints=node.create_publisher(JointState,'/joint_states',10)
        paths=node.create_publisher(DisplayTrajectory,'/display_planned_path',10)
        names=[f'{side}_joint{i}' for side in ('left','right') for i in range(1,8)]
        values=[.2]*7+[.6]*7
        current=DisplayTrajectory(model_id='dual_nero')
        current.trajectory_start.joint_state=JointState(name=names,position=values)
        t=RobotTrajectory();t.joint_trajectory.joint_names=names[7:]
        t.joint_trajectory.points=[JointTrajectoryPoint(positions=values[7:]),JointTrajectoryPoint(positions=[.61]*7)]
        current.trajectory=[t]
        old=copy.deepcopy(current);old.trajectory_start.joint_state.position=[0.]*14
        old.trajectory[0].joint_trajectory.points[0].positions=[0.]*7
        try:
            deadline=time.monotonic()+1.5
            while time.monotonic()<deadline:
                msg=JointState(name=names,position=values);msg.header.stamp=node.get_clock().now().to_msg();joints.publish(msg)
                rclpy.spin_once(node,timeout_sec=.02)
            paths.publish(current)
            for _ in range(20):
                paths.publish(old)
                msg=JointState(name=names,position=values);msg.header.stamp=node.get_clock().now().to_msg();joints.publish(msg)
                rclpy.spin_once(node,timeout_sec=.02)
            deadline=time.monotonic()+.4
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
            self.assertEqual(len(received),1)
            self.assertEqual(list(received[0].trajectory[0].joint_trajectory.points[0].positions),[.6]*7)
            self.assertTrue(sync,'GUI start-state update must be requested')
        finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
