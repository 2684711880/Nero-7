"""Binary target switching and official gripper-only callback; no hardware."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from joycon_move_p_dual import Teleop
from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class GripperTests(unittest.TestCase):
    def teleop(self):
        return SimpleNamespace(gripper_open_target={'left': None, 'right': None},
                               gripper={'left': Mock(), 'right': Mock()})

    def test_explicit_independent_states_and_preview(self):
        node = self.teleop()
        widths = [Teleop.toggle_gripper(node, 'left', True).position[0] for _ in range(4)]
        self.assertEqual(widths, [0., .1, 0., .1])
        self.assertIsNone(node.gripper_open_target['right'])
        self.assertEqual(Teleop.toggle_gripper(node, 'right', False).position[0], 0.)
        node.gripper['right'].publish.assert_not_called()

    def test_only_gripper_reaches_official_callback(self):
        teleop = self.teleop()
        sdk, gripper = Mock(), Mock()
        node = SimpleNamespace(agx_arm=sdk, gripper=gripper, gripper_default_effort=1.,
                               arm_joint_names=[f'joint{i}' for i in range(1, 8)],
                               _check_can_control=lambda: True,
                               _safe_get_value=lambda array, index, default=0.: array[index] if index < len(array) else default)
        node._control_arm_joints = lambda pos: AgxArmRosNode._control_arm_joints(node, pos)
        node._control_gripper_joint = lambda pos, effort: AgxArmRosNode._control_gripper_joint(node, pos, effort)
        node._control_hand_joints = lambda pos: None
        for expected in (0., .1, 0.):
            message = Teleop.toggle_gripper(teleop, 'left', True)
            AgxArmRosNode._joint_states_callback(node, message)
            gripper.move.assert_called_with(width=expected, force=1.)
        sdk.move_j.assert_not_called()
        sdk.move_js.assert_not_called()


if __name__ == '__main__':
    unittest.main()
