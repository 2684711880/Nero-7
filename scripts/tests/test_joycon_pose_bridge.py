#!/usr/bin/env python3
"""Verify simulated dual-arm joystick-to-official-callback path without CAN."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from joycon_preview import mount_rotations


def main():
    rotations = mount_rotations()
    for side in ('left', 'right'):
        for world_delta in ([0.001, 0, 0], [0, -0.001, 0]):
            # Use a nonzero start and nontrivial orientation to catch loss of pose.
            start = np.array([-0.4, 0.05, 0.2])
            base_delta = rotations[side].inv().apply(world_delta)
            np.testing.assert_allclose(rotations[side].apply(base_delta), world_delta, atol=1e-12)
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = start + base_delta
            q = Rotation.from_euler('xyz', [0.1, -0.2, 0.3]).as_quat()
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            sdk = Mock()
            sdk.get_tcp2flange_pose.side_effect = lambda value: value
            node = SimpleNamespace(agx_arm=sdk, _check_can_control=lambda: True, is_mit_mode=True)
            node._create_pose_cmd = lambda value: AgxArmRosNode._create_pose_cmd(node, value)
            AgxArmRosNode._move_p_callback(node, pose)
            sdk.move_p.assert_called_once()
            actual = sdk.move_p.call_args.args[0]
            np.testing.assert_allclose(actual[:3], start + base_delta, atol=1e-12)
            np.testing.assert_allclose(actual[3:], [0.1, -0.2, 0.3], atol=1e-12)
            assert not node.is_mit_mode
            print(f'{side} world_delta={world_delta}: official move_p callback OK')
    print('PASS: mock SDK only, no CAN or robot motion')


if __name__ == '__main__':
    main()
