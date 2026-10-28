"""Exercise the exact camera CLI used by launch_ros, without opening hardware."""
from pathlib import Path
import sys
import subprocess
import unittest
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).parents[1]))
from handeye_rviz_inputs import parse_cli
from start_pointing import pointing_environment,stop_launch,ROOT


class CameraCLITests(unittest.TestCase):
    def test_launch_ros_arguments_are_removed_before_application_parsing(self):
        a=parse_cli(['pointing_camera','--can','can1','--serial','262622274345',
                     '--session','/tmp/session','--no-robot-tf','--ros-args','-r','__ns:=/test'])
        self.assertEqual(a.can,'can1')
        self.assertEqual(a.serial,'262622274345')
        self.assertEqual(a.session,Path('/tmp/session'))
        self.assertTrue(a.no_robot_tf)

    def test_terminal_entry_still_accepts_the_same_flags(self):
        a=parse_cli(['camera','--can','can1','--session','/tmp/session','--check-seconds','3'])
        self.assertEqual(a.check_seconds,3.)
        self.assertFalse(a.no_robot_tf)

    def test_camera_only_does_not_require_a_can_interface(self):
        a=parse_cli(['camera','--camera-only','--session','/tmp/session'])
        self.assertTrue(a.camera_only)
        self.assertIsNone(a.can)

    def test_real_stack_gets_enough_cyclonedds_participant_slots(self):
        original=dict(RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
        env=pointing_environment(original)
        self.assertNotIn('CYCLONEDDS_URI',original)
        self.assertEqual(env['CYCLONEDDS_URI'],(ROOT/'config/cyclonedds_pointing.xml').as_uri())
        tree=ET.parse(ROOT/'config/cyclonedds_pointing.xml')
        namespace={'c':'https://cdds.io/config'}
        count=int(tree.findtext('.//c:MaxAutoParticipantIndex',namespaces=namespace))
        self.assertGreaterEqual(count,32)

    def test_other_rmw_settings_are_preserved(self):
        env=pointing_environment(dict(RMW_IMPLEMENTATION='rmw_fastrtps_cpp'))
        self.assertNotIn('CYCLONEDDS_URI',env)
        self.assertEqual(env['RMW_IMPLEMENTATION'],'rmw_fastrtps_cpp')

    def test_shutdown_reaps_its_group_and_leaves_unrelated_process_alone(self):
        child=subprocess.Popen([sys.executable,'-u','-c',
            'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); print("ready"); time.sleep(30)'],
            start_new_session=True,stdout=subprocess.PIPE,text=True)
        unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'])
        try:
            self.assertEqual(child.stdout.readline().strip(),'ready')
            stop_launch(child,grace=.1)
            self.assertIsNotNone(child.poll())
            self.assertIsNone(unrelated.poll())
        finally:
            if child.poll() is None:child.kill();child.wait()
            child.stdout.close()
            unrelated.terminate();unrelated.wait()


if __name__=='__main__':unittest.main()
