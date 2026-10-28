"""Calibrated FK, model resources and planning/execution separation."""
from pathlib import Path
import sys
import copy
import tempfile
import unittest
import xml.etree.ElementTree as ET
from urllib.parse import urlparse,unquote
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]))
from pointing_model import compose,ROOT
from handeye_charuco import FK
from start_pointing import model_gripper_center_mm


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.offset=np.array(model_gripper_center_mm())/1000.
        cls.initial=[.1,.8,-.1,.4,.2,.1,-.2]
        cls.cfg,cls.controllers=compose(cls.offset,cls.initial)
        cls.robot=ET.fromstring(cls.cfg['robot_description'])

    def test_gripper_centre_is_model_value(self):
        np.testing.assert_allclose(self.offset,[0,0,.1413])

    def test_every_mesh_resource_exists(self):
        meshes=self.robot.findall('.//mesh')
        self.assertGreater(len(meshes),20)
        for mesh in meshes:
            uri=urlparse(mesh.get('filename'))
            self.assertEqual(uri.scheme,'file')
            self.assertTrue(Path(unquote(uri.path)).is_file())

    def test_planning_model_keeps_calibrated_seven_axis_fk(self):
        # Strip one arm's naming prefix, retaining its exact local chain.
        local=ET.Element('robot',name='local')
        for e in self.robot:
            if e.tag not in ('link','joint') or not e.get('name','').startswith('left_'):continue
            if e.get('name')=='left_arm_joint':continue
            e=copy.deepcopy(e)
            e.set('name',e.get('name').removeprefix('left_'))
            for child in e:
                if child.tag in ('parent','child'):child.set('link',child.get('link').removeprefix('left_'))
            local.append(e)
        with tempfile.NamedTemporaryFile('w',suffix='.urdf') as f:
            f.write(ET.tostring(local,encoding='unicode'));f.flush()
            tip=FK(f.name,tip='point_tcp')
            original=FK(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf')
            for q in np.random.default_rng(12).uniform(-1,1,(20,7)):
                expected=original(q).copy()
                expected[:3,3]=(expected@np.r_[self.offset,1.])[:3]
                np.testing.assert_allclose(tip(q),expected,atol=1e-12)

    def test_controller_buffer_uses_measured_initial_state_and_seven_joints(self):
        control=self.robot.find('ros2_control')
        self.assertEqual(control.findtext('hardware/plugin'),'mock_components/GenericSystem')
        joints=control.findall('joint');self.assertEqual(len(joints),7)
        np.testing.assert_allclose([float(j.find("state_interface[@name='position']/param").text) for j in joints],self.initial)
        self.assertEqual(self.cfg['moveit_simple_controller_manager']['left_arm_controller']['joints'],[f'left_joint{i}' for i in range(1,8)])

    def test_semantic_model_preserves_nonadjacent_collision_checks(self):
        s=ET.fromstring(self.cfg['robot_description_semantic'])
        self.assertEqual(s.find("group[@name='left_arm']/chain").get('tip_link'),'left_point_tcp')
        self.assertEqual(s.find("group[@name='right_arm']/chain").get('tip_link'),'right_point_tcp')
        pairs=[{e.get('link1'),e.get('link2')} for e in s.findall('disable_collisions')]
        self.assertIn({'left_link1','left_link2'},pairs)
        self.assertNotIn({'left_link1','left_link5'},pairs)
        self.assertNotIn({'left_link5','right_link5'},pairs)

    def test_both_seven_axis_chains_are_present_with_official_mounts(self):
        from pointing_model import mount_matrix
        for side,y in [('left',.1),('right',-.1)]:
            self.assertEqual(len([j for j in self.robot.findall('joint') if j.get('type')=='revolute' and j.get('name').startswith(side+'_')]),7)
            np.testing.assert_allclose(mount_matrix(side)[:3,3],[-.002,y,.59])
        self.assertEqual(self.cfg['moveit_simple_controller_manager']['controller_names'],['left_arm_controller'])


if __name__=='__main__':unittest.main()
