import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from mobile_dual_nero.model import compose, gazebo_model_paths


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
        self.assertEqual(len(controlled), 26)
        self.assertEqual(len({j.get('name') for j in controlled}), 26)
        for item in s.findall('disable_collisions'):
            self.assertIn(item.get('link1'), links)
            self.assertIn(item.get('link2'), links)
        self.assertEqual(s.find('virtual_joint').get('type'), 'planar')

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
