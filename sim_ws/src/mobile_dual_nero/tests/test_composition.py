import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from mobile_dual_nero.model import compose, gazebo_model_paths, RELAXED_ARM


class CompositionTest(unittest.TestCase):
    def test_one_movable_tree_and_control_system(self):
        urdf, srdf = compose()
        r, s = ET.fromstring(urdf), ET.fromstring(srdf)
        links = [l.get('name') for l in r.findall('link')]
        joints = [j.get('name') for j in r.findall('joint')]
        self.assertEqual(len(links), len(set(links)))
        self.assertEqual(len(joints), len(set(joints)))
        children = [j.find('child').get('link') for j in r.findall('joint')]
        self.assertEqual(set(links)-set(children), {'base_link'})
        self.assertNotIn('world', links)
        self.assertEqual(len(r.findall('ros2_control')), 1)
        self.assertEqual(len(r.findall('.//plugin')), 2)  # hardware class plus Gazebo plugin
        controlled = r.findall('ros2_control/joint')
        self.assertEqual(len(controlled), 27)
        self.assertEqual(len({j.get('name') for j in controlled}), 27)
        for item in s.findall('disable_collisions'):
            self.assertIn(item.get('link1'), links)
            self.assertIn(item.get('link2'), links)
        self.assertEqual(s.find('virtual_joint').get('type'), 'planar')

    def test_relaxed_initial_state_matches_named_pose_and_limits(self):
        r, s = map(ET.fromstring, compose())
        state = s.find("group_state[@name='relaxed'][@group='both_arms']")
        self.assertEqual(len(state.findall('joint')), 14)
        for joint in state.findall('joint'):
            name, angle = joint.get('name'), float(joint.get('value'))
            limit = r.find(f"joint[@name='{name}']/limit")
            self.assertLessEqual(float(limit.get('lower')), angle)
            self.assertLessEqual(angle, float(limit.get('upper')))
            self.assertEqual(float(r.findtext(f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param[@name='initial_value']")), angle)
        legacy = ET.fromstring(compose(startup_arm_pose='home')[0])
        for side in ('left', 'right'):
            for i in range(1, 8):
                self.assertEqual(float(legacy.findtext(f"ros2_control/joint[@name='{side}_joint{i}']/state_interface[@name='position']/param[@name='initial_value']")), 0.0)

    def test_mid360_mount_rotation_and_override(self):
        import math
        from mobile_dual_nero.mid360 import mount_pose
        for preset, roll in [('chassis_up', 0.0), ('chest_down', math.pi)]:
            r = ET.fromstring(compose(mid360_mount=preset)[0])
            origin = r.find("joint[@name='mid360_joint']/origin")
            self.assertAlmostEqual(float(origin.get('rpy').split()[0]), roll)
            self.assertIsNotNone(r.find("gazebo/plugin[@name='nero_mid360']"))
            self.assertIsNotNone(r.find("link[@name='mid360_link']/collision"))
            self.assertEqual(r.findtext("gazebo[@reference='mid360_joint']/preserveFixedJoint"), 'true')
        self.assertEqual(mount_pose('chassis_up', '.3 0 .4', '0 .1 0'), ([.3, 0., .4], [0., .1, 0.]))
        with self.assertRaises(ValueError):
            mount_pose('chassis_up', 'nan 0 0')

    def test_lift_guide_mass_center_and_rigid_body_inertia(self):
        r = ET.fromstring(compose()[0])
        rail = r.find("link[@name='upper_body_lift_rail']")
        inertia = rail.find('inertial')
        self.assertEqual(inertia.find('origin').get('xyz'), rail.find('collision/origin').get('xyz'))
        mass = float(inertia.find('mass').get('value'))
        x, y, z = map(float, rail.find('collision/geometry/box').get('size').split())
        values = [float(inertia.find('inertia').get(k)) for k in ('ixx','iyy','izz')]
        expected = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
        for actual, value in zip(values, expected): self.assertAlmostEqual(actual, value)
        self.assertGreater(min(values), 0)
        self.assertLessEqual(max(values), sum(values)-max(values))

    def test_mount_parameter(self):
        r = ET.fromstring(compose('0.02 0 0.31')[0])
        origin = r.find("joint[@name='air_mount_joint']/origin")
        self.assertAlmostEqual(float(origin.get('xyz').split()[2]), .304)

    def test_gazebo_client_resolves_every_mesh(self):
        root = ET.fromstring(compose()[0])
        search = [Path(p) for p in gazebo_model_paths()]
        for mesh in root.findall('.//mesh'):
            uri = mesh.get('filename')
            relative = uri.removeprefix('package://').removeprefix('model://')
            self.assertTrue(any((path / relative).is_file() for path in search), uri)


if __name__ == '__main__':
    unittest.main()
