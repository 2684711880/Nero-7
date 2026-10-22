"""Lifecycle boundaries use real OS process state and a foreign TCP listener."""
import importlib.util
import os
from pathlib import Path
import socket
import subprocess
import sys
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'store_mapping_session.py'
spec = importlib.util.spec_from_file_location('session', SCRIPT)
session = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session)
ROOT = SCRIPT.parents[1]


class SessionTest(unittest.TestCase):
    def test_identity_requires_project_domain_endpoint_and_exact_entry(self):
        info = dict(cwd=ROOT/'sim_ws', env={b'ROS_DOMAIN_ID':b'89',
                    b'GAZEBO_MASTER_URI':b'http://127.0.0.1:11489'},
                    argv=['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch',
                          'nero_mid360_sim', 'store_superlio.launch.py', 'ceiling:=false'])
        self.assertTrue(session.matching_launch(info, ROOT, '89', 'http://127.0.0.1:11489'))
        self.assertFalse(session.matching_launch(info, ROOT/'other', '89', 'http://127.0.0.1:11489'))
        self.assertFalse(session.matching_launch(info, ROOT, '90', 'http://127.0.0.1:11489'))
        self.assertFalse(session.matching_launch(info, ROOT, '89', 'http://127.0.0.1:11490'))
        info['argv'][4]='another.launch.py'
        self.assertFalse(session.matching_launch(info, ROOT, '89', 'http://127.0.0.1:11489'))

    def test_different_process_start_time_cannot_be_signalled(self):
        info = session.process_info(os.getpid())
        self.assertTrue(session.alive(info))
        info['start'] = 'not-this-process'
        self.assertFalse(session.alive(info))

    def test_foreign_listener_is_preserved(self):
        # An occupied Gazebo endpoint with no matching project launch must fail safely.
        with socket.socket() as server:
            server.bind(('127.0.0.1',0));server.listen()
            port=server.getsockname()[1]
            env={**os.environ,'ROS_DOMAIN_ID':'189','GAZEBO_MASTER_URI':f'http://127.0.0.1:{port}'}
            result=subprocess.run([sys.executable,str(SCRIPT),'--project-root',str(ROOT)],
                                  env=env,capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,1)
            self.assertIn('其他仿真',result.stderr)
            self.assertTrue(session.port_busy('127.0.0.1',port))


if __name__ == '__main__':
    unittest.main()
