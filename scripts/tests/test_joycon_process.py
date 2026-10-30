"""Real process-tree cleanup regression; dummy processes only, no ROS or CAN."""
from pathlib import Path
import subprocess
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from joycon_process import group_alive, stop_group


class ProcessTests(unittest.TestCase):
    def test_orphan_child_ignoring_int_and_term_is_removed(self):
        code = '''
import os, signal, time
pid = os.fork()
if pid == 0:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    print(os.getpid(), flush=True)
    while True: time.sleep(1)
else:
    signal.signal(signal.SIGINT, lambda *args: exit(0))
    while True: time.sleep(1)
'''
        process = subprocess.Popen([sys.executable, '-u', '-c', code], start_new_session=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        other = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                 start_new_session=True, stderr=subprocess.DEVNULL)
        try:
            self.assertTrue(process.stdout.readline().strip().isdigit())
            stop_group(process, grace=.15)
            self.assertFalse(group_alive(process.pid))
            self.assertIsNone(other.poll())
        finally:
            stop_group(process, grace=.05)
            stop_group(other, grace=.05)
            process.stdout.close()


if __name__ == '__main__':
    unittest.main()
