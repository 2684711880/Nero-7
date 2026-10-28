"""Orbbec input startup regression tests using real ROS Node, no USB or CAN."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from handeye_orbbec_inputs import Inputs, ROOT


class OrbbecStartupTest(unittest.TestCase):
    def test_sdk_context_does_not_replace_ros_context(self):
        rclpy.init()
        instance = Inputs.__new__(Inputs)
        with tempfile.TemporaryDirectory() as directory:
            args = SimpleNamespace(session=Path(directory), camera_only=True,
                                   serial=None, can='can1',
                                   urdf=ROOT / 'config/gravity/nero_standard_gripper_no_camera.urdf')
            try:
                with patch('handeye_orbbec_inputs.Context') as sdk_context:
                    sdk_context.return_value.query_devices.return_value.get_count.return_value = 0
                    with self.assertRaisesRegex(RuntimeError, 'No Orbbec camera detected'):
                        instance.__init__(args)
                    sdk_context.return_value.query_devices.assert_called_once()
                    self.assertIs(instance.context, rclpy.get_default_context())
            finally:
                instance.close()
                instance.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
