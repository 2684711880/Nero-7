"""Park-only disable checks with simulated SDK feedback; no CAN."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import math
import sys
import time
import unittest
from unittest.mock import patch

scripts=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(scripts))
spec=importlib.util.spec_from_file_location('disable_at_park',scripts/'disable_at_park.py')
disable=importlib.util.module_from_spec(spec)
spec.loader.exec_module(disable)


def msg(value): return NS(msg=value,timestamp=time.time())


class Arm:
    def __init__(self,angles=None):
        self.q=list(disable.PARK if angles is None else angles)
        self._parser=NS(**{group:msg(None) for group in ('joint_12','joint_34','joint_56','joint_7')})
        self.calls=0
        self.enabled=True
        self.closed=False
    def connect(self): pass
    def disconnect(self): self.closed=True
    def get_joint_angles(self): return msg(self.q)
    def get_driver_states(self,index): return msg(NS(foc_status=NS(driver_enable_status=self.enabled)))
    def disable(self,timeout):
        self.calls+=1
        self.enabled=False
        return True


class DisableTests(unittest.TestCase):
    def run_case(self,arms,args=[]):
        def factory(cfg): return arms[cfg['channel']]
        with patch.object(sys,'argv',['disable']+args), \
             patch.object(disable,'create_agx_arm_config',side_effect=lambda **kw:kw), \
             patch.object(disable.AgxArmFactory,'create_arm',side_effect=factory), \
             patch.object(disable.time,'sleep'):
            return disable.main()

    def test_both_park_disable_and_confirm(self):
        arms={'can1':Arm(),'can0':Arm()}
        self.assertEqual(self.run_case(arms),0)
        self.assertTrue(all(arm.calls==1 and arm.closed for arm in arms.values()))

    def test_right_outside_means_neither_is_disabled(self):
        arms={'can1':Arm(),'can0':Arm([0.0]*7)}
        self.assertEqual(self.run_case(arms),1)
        self.assertEqual([arm.calls for arm in arms.values()],[0,0])

    def test_left_selection_only(self):
        arm=Arm()
        self.assertEqual(self.run_case({'can1':arm},['left']),0)
        self.assertEqual(arm.calls,1)

    def test_nonzero_other_joint_blocks(self):
        q=list(disable.PARK)
        q[5]=math.radians(4)
        arm=Arm(q)
        self.assertEqual(self.run_case({'can0':arm},['right']),1)
        self.assertEqual(arm.calls,0)

    def test_stale_individual_frame_blocks(self):
        arm=Arm()
        arm._parser.joint_56.timestamp-=1
        with self.assertRaises(disable.FeedbackPending): disable.check_park(arm,'left')
        self.assertEqual(arm.calls,0)

    def test_missing_disable_feedback_is_not_false(self):
        arm=Arm()
        arm.get_driver_states=lambda i:None
        with self.assertRaises(disable.FeedbackPending): disable.enable_flags(arm)


if __name__=='__main__': unittest.main()
