"""Exercise the empty-pipeline symptom left behind by a USB re-enumeration."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).parents[1]))
from handeye_rviz_inputs import Inputs


class CameraReconnectTests(unittest.TestCase):
    def node(self):
        node = SimpleNamespace(pipe=Mock(), last_capture=10., last_restart=float('-inf'),
                               camera_serial='262622274345', start_camera=Mock(), get_logger=Mock())
        node.recover_camera = lambda now: Inputs.recover_camera(node, now)
        return node

    def test_empty_pipeline_reopens_without_restarting_robot_inputs(self):
        node=self.node()
        pipe=node.pipe
        pipe.poll_for_frames.return_value=None
        with patch('handeye_rviz_inputs.time.monotonic', return_value=14.):
            Inputs.on_frame(node)
        pipe.stop.assert_called_once()
        node.start_camera.assert_called_once()
        self.assertEqual(node.camera_serial,'262622274345')

    def test_disconnected_camera_retries_at_bounded_rate(self):
        node=self.node()
        node.start_camera.side_effect=RuntimeError('device absent')
        node.recover_camera(14.)
        self.assertIsNone(node.pipe)
        node.recover_camera(14.1)
        node.recover_camera(18.9)
        self.assertEqual(node.start_camera.call_count,1)
        node.recover_camera(19.1)
        self.assertEqual(node.start_camera.call_count,2)

    def test_normal_frame_interval_does_not_restart(self):
        node=self.node()
        node.recover_camera(10.2)
        node.start_camera.assert_not_called()
        node.pipe.stop.assert_not_called()

if __name__=='__main__':unittest.main()
