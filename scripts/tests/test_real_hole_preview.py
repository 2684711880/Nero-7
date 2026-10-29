import sys
import unittest
from pathlib import Path

from moveit_msgs.msg import RobotState, AttachedCollisionObject

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_hole_preview import current_display


class PreviewTest(unittest.TestCase):
    def test_reset_preserves_both_arms_grippers_and_attached_board(self):
        state=RobotState()
        state.joint_state.name=[f'{s}_joint{i}' for s in ('left','right') for i in range(1,8)]+['left_gripper','right_gripper']
        state.joint_state.position=[.11*i for i in range(14)]+[.023,.033]
        board=AttachedCollisionObject();board.link_name='left_gripper_base';board.object.id='board'
        state.attached_collision_objects=[board]
        msg=current_display(state)
        self.assertEqual(msg.trajectory_start.attached_collision_objects[0].object.id,'board')
        path=msg.trajectory[0].joint_trajectory
        self.assertEqual(len(path.points),1)
        self.assertEqual(list(path.points[0].positions),list(state.joint_state.position))
        self.assertEqual(list(path.joint_names),list(state.joint_state.name))

    def test_missing_feedback_does_not_fall_back_to_zero_pose(self):
        with self.assertRaises(RuntimeError):current_display(RobotState())


if __name__=='__main__':unittest.main()
