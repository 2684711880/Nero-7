"""Launcher/configuration checks only: no CAN and no physical arm execution."""
import importlib.util
from pathlib import Path
import signal
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch


def load(name):
    path=Path(__file__).resolve().parents[1]/(name+'.py')
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


launcher=load('run_nero_gravity')


class LauncherTests(unittest.TestCase):
    def test_default_is_both(self):
        with patch.object(sys,'argv',['gravity']),patch.object(launcher,'run_both',return_value=0) as run:
            self.assertEqual(launcher.main(),0)
            run.assert_called_once()

    def test_single_selects_only_one_worker(self):
        for side in ('left','right'):
            for arguments in ([side],['--arm',side]):
                with patch.object(sys,'argv',['gravity']+arguments),patch.object(launcher.os,'execv',side_effect=SystemExit) as execute:
                    with self.assertRaises(SystemExit): launcher.main()
                    self.assertEqual(execute.call_args.args[1][-2:],['--side',side])

    def test_one_child_exits_other_receives_official_ctrl_c(self):
        first=Mock()
        first.poll.return_value=1
        second=Mock()
        second.poll.return_value=None
        with patch.object(launcher.subprocess,'Popen',side_effect=[first,second]) as spawn:
            self.assertEqual(launcher.run_both(),1)
        self.assertEqual([call.args[0][-1] for call in spawn.call_args_list],['left','right'])
        self.assertTrue(all(call.kwargs['start_new_session'] for call in spawn.call_args_list))
        second.send_signal.assert_called_once_with(signal.SIGINT)
        first.wait.assert_called_once()
        second.wait.assert_called_once()

    def test_interrupt_stops_both_children(self):
        children=[Mock(),Mock()]
        for child in children: child.poll.return_value=None
        with patch.object(launcher.subprocess,'Popen',side_effect=children),patch.object(launcher.time,'sleep',side_effect=KeyboardInterrupt):
            self.assertEqual(launcher.run_both(),130)
        for child in children:
            child.send_signal.assert_called_once_with(signal.SIGINT)
            child.wait.assert_called_once()

    def test_worker_uses_matching_can_and_side_rotation(self):
        worker=load('nero_gc_official_configured')
        for side,can,roll in [('left','can1',-1.57),('right','can0',1.57)]:
            robot=Mock(joint_nums=7)
            robot.enable.return_value=True
            robot.get_joint_angles.return_value=NS(msg=[0.0]*7)
            robot.get_motor_states.return_value=None
            robot.move_mit.side_effect=KeyboardInterrupt
            dynamics=Mock()
            dynamics.inverse_dynamics.return_value=worker.np.zeros(7)
            with patch.object(sys,'argv',['worker','--side',side]), \
                 patch.object(worker,'AgxPinocchio',return_value=dynamics), \
                 patch.object(worker,'create_agx_arm_config',return_value={}) as config, \
                 patch.object(worker.AgxArmFactory,'create_arm',return_value=robot):
                worker.main()
            self.assertEqual(config.call_args.kwargs['channel'],can)
            rotation=dynamics.inverse_dynamics.call_args.args[3]
            expected=worker.R.from_euler('xyz',[roll,-1.57,0.0]).as_matrix()
            worker.np.testing.assert_allclose(rotation,expected)
            robot.move_j.assert_called_once_with([0.0]*7)


if __name__=='__main__': unittest.main()
