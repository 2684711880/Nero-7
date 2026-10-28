"""No CAN: validate official model composition and left/right command routing."""
import unittest
from types import SimpleNamespace as NS
from pathlib import Path
import xml.etree.ElementTree as ET
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from nero_dual_bringup.model import compose
from nero_dual_bringup.bridge import JointBridge, renamed
from nero_dual_bringup.scene import scene_objects


def state():
    msg = JointState()
    msg.name = [side+'_'+f'joint{i}' for side in ('left', 'right') for i in range(1,8)]
    msg.name += ['left_gripper', 'right_gripper']
    msg.position = [i/100 for i in range(len(msg.name))]
    return msg


class WiringTests(unittest.TestCase):
    def test_real_scene_frame_and_cabinet_geometry(self):
        config, _ = compose(.28)
        robot = ET.fromstring(config['robot_description'])
        origin = robot.find("joint[@name='world_joint']/origin")
        self.assertEqual([float(v) for v in origin.get('xyz').split()], [0, 0, .28])
        for side in ('left', 'right'):
            mount = robot.find(f"joint[@name='{side}_arm_joint']/origin")
            self.assertAlmostEqual(.28+float(mount.get('xyz').split()[2]), .87)
        objects = {name: (size, xyz) for name, size, xyz in scene_objects()}
        size, xyz = objects['front_cabinet']
        self.assertAlmostEqual(xyz[2]+size[2]/2, .60)
        self.assertAlmostEqual(xyz[0]-size[0]/2, .50)
        semantic = ET.fromstring(config['robot_description_semantic'])
        import math
        for group, sides in [('left_arm', ('left',)), ('right_arm', ('right',)), ('both_arms', ('left', 'right'))]:
            park = semantic.find(f"group_state[@name='park'][@group='{group}']")
            self.assertIsNotNone(park)
            actual = {j.get('name'): float(j.get('value')) for j in park.findall('joint')}
            for side in sides:
                for i, target in enumerate((0, 85, 0, 5, 0, 0, 0), 1):
                    self.assertAlmostEqual(actual[f'{side}_joint{i}'], math.radians(target))

    def harness(self):
        outputs = {side: [] for side in ('left','right')}
        node = NS(active={(s,p):False for s in outputs for p in ('arm','gripper')},
            outputs={s:NS(publish=outputs[s].append) for s in outputs},
            get_parameter=lambda name: NS(value=1.7 if name.startswith('right') else .8))
        return node, outputs

    def test_idle_zero_commands_do_not_reach_drivers(self):
        node, outputs = self.harness()
        JointBridge.commands(node, state())
        self.assertEqual(outputs, {'left': [], 'right': []})

    def test_left_only_does_not_command_right(self):
        node, outputs = self.harness()
        node.active['left','arm'] = True
        JointBridge.commands(node, state())
        self.assertEqual(outputs['left'][0].name, [f'joint{i}' for i in range(1,8)])
        self.assertEqual(outputs['right'], [])

    def test_gripper_only_does_not_move_arm(self):
        node, outputs = self.harness()
        node.active['right','gripper'] = True
        JointBridge.commands(node, state())
        self.assertEqual(outputs['right'][0].name, ['gripper'])
        self.assertEqual(outputs['left'], [])

    def test_force_routed_only_to_active_gripper(self):
        node, outputs = self.harness()
        node.active['right','gripper'] = True
        node.active['right','arm'] = True
        JointBridge.commands(node, state())
        command = outputs['right'][0]
        self.assertEqual(command.name[-1], 'gripper')
        self.assertEqual(list(command.effort), [0.0]*7+[1.7])
        self.assertEqual(outputs['left'], [])

    def test_invalid_force_rejected(self):
        for value in (0., .49, 3.01, float('nan'), float('inf'), True, '1.0'):
            self.assertFalse(JointBridge.validate_force([NS(name='right_gripper_force',value=value)]).successful)
        for value in (.5, 1.7, 3.):
            self.assertTrue(JointBridge.validate_force([NS(name='right_gripper_force',value=value)]).successful)

    def test_partial_arm_is_never_padded_with_zero(self):
        node, outputs = self.harness()
        node.active['right','arm'] = True
        msg = state()
        msg.position = msg.position[:8]
        JointBridge.commands(node, msg)
        self.assertEqual(outputs['right'], [])

    def test_feedback_prefix_and_width_unchanged(self):
        msg = JointState(name=['joint1','gripper'], position=[0.4,0.06])
        result = renamed(msg, 'right', True)
        self.assertEqual(result.name, ['right_joint1','right_gripper'])
        self.assertEqual(list(result.position), [0.4,0.06])

    def test_model_and_semantic_resources(self):
        config, controllers = compose()
        robot = ET.fromstring(config['robot_description'])
        links = {x.get('name') for x in robot.findall('link')}
        joints = {x.get('name') for x in robot.findall('joint')}
        for joint in robot.findall('joint'):
            self.assertIn(joint.find('parent').get('link'),links)
            self.assertIn(joint.find('child').get('link'),links)
        self.assertEqual(len([j for j in robot.findall('joint') if j.get('type')=='revolute']),14)
        for mesh in robot.findall('.//mesh'):
            pkg, rel = mesh.get('filename').removeprefix('package://').split('/',1)
            self.assertTrue((Path(get_package_share_directory(pkg))/rel).exists(),mesh.attrib)
        semantic=ET.fromstring(config['robot_description_semantic'])
        for item in semantic.findall('disable_collisions'):
            self.assertIn(item.get('link1'),links)
            self.assertIn(item.get('link2'),links)
        self.assertIn('both_arms',{x.get('name') for x in semantic.findall('group')})
        for name, data in config['moveit_simple_controller_manager'].items():
            if isinstance(data,dict):
                self.assertTrue(set(data['joints'])<=joints)


if __name__=='__main__':
    unittest.main()
