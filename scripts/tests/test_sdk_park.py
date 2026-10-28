"""Official SDK call wiring only; no CAN connection."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import time
import sys
import unittest

path=Path(__file__).resolve().parents[1]/'move_to_park.py'
spec=importlib.util.spec_from_file_location('sdk_park',path)
park=importlib.util.module_from_spec(spec)
spec.loader.exec_module(park)


class FakeArm:
    def __init__(self): self.targets=[]; self.closed=False
    def connect(self): pass
    def enable(self): return True
    def set_speed_percent(self,value): self.speed=value
    def move_j(self,target): self.targets.append(target)
    def get_joint_angles(self): return NS(msg=self.targets[-1],timestamp=time.time())
    def disconnect(self): self.closed=True


class ParkTests(unittest.TestCase):
    def run_case(self,args,channels):
        arms=[]
        selected=[]
        def create(**config):
            selected.append(config['channel'])
            return config
        def factory(config):
            robot=FakeArm()
            arms.append(robot)
            return robot
        with patch.object(sys,'argv',['park']+args), \
             patch.object(park,'create_agx_arm_config',side_effect=create), \
             patch.object(park.AgxArmFactory,'create_arm',side_effect=factory), \
             patch.object(park.time,'sleep'):
            self.assertEqual(park.main(),0)
        self.assertEqual(selected,channels)
        for robot in arms:
            self.assertEqual(robot.targets,[park.PARK])
            self.assertEqual(robot.speed,10)
            self.assertTrue(robot.closed)

    def test_default_both(self): self.run_case([],['can1','can0'])
    def test_left_only(self): self.run_case(['left'],['can1'])
    def test_right_only(self): self.run_case(['right'],['can0'])


if __name__=='__main__': unittest.main()
