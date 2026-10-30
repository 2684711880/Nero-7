"""Manual stop regression using dummy processes, with no ROS/CAN connection."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import joycon_stop


class StopTests(unittest.TestCase):
    def test_manual_stop_cleans_standalone_teleop_and_residual_driver_only(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / 'project'
            teleop = root / 'scripts/joycon_move_p_dual.py'
            driver = root / 'hardware_ws/install/agx_arm_ctrl/lib/agx_arm_ctrl/agx_arm_ctrl_single'
            other = Path(folder) / 'other/scripts/joycon_move_p_dual.py'
            normal = "import time\nprint('ready',flush=True)\nwhile True: time.sleep(1)\n"
            stubborn = "import signal\nsignal.signal(signal.SIGINT,signal.SIG_IGN)\nsignal.signal(signal.SIGTERM,signal.SIG_IGN)\n" + normal
            for path, content in ((teleop, normal), (driver, stubborn), (other, normal)):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
            processes = []
            try:
                for path, args in ((teleop, ['--execute']), (driver, ['--ros-args', '-r', '__ns:=/left']),
                                   (other, ['--execute'])):
                    process = subprocess.Popen([sys.executable, str(path), *args], start_new_session=True,
                                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                    processes.append(process)
                    self.assertEqual(process.stdout.readline().strip(), 'ready')
                with patch.object(joycon_stop, 'ROOT', root):
                    self.assertEqual(len(joycon_stop.find_processes()), 2)
                    joycon_stop.stop_all()
                    self.assertFalse(joycon_stop.find_processes())
                for process in processes[:2]:
                    process.wait(timeout=2)
                self.assertIsNone(processes[2].poll())
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.kill()
                    process.wait(timeout=2)
                    process.stdout.close()


if __name__ == '__main__':
    unittest.main()
