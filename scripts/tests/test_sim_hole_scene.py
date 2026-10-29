import json,sys,unittest,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'sim_ws/src/dual_nero_moveit_config/launch'))
from hole_scene import add_hole_scene

class SceneTest(unittest.TestCase):
 def test_hole_camera_and_tool_geometry(self):
  robot=ET.Element('robot');semantic=ET.Element('robot');cal=json.loads((ROOT/'config/handeye/right/d405.json').read_text());add_hole_scene(robot,semantic,cal)
  plate=robot.find("link[@name='sim_hole_plate']");collisions=plate.findall('collision');self.assertEqual(len(collisions),36)
  first=plate.find("collision[@name='sector_0_collision']");centre=np.fromstring(first.find('origin').get('xyz'),sep=' ');size=np.fromstring(first.find('geometry/box').get('size'),sep=' ')
  self.assertAlmostEqual(centre[0]-size[0]/2,.045)
  left=plate.find("collision[@name='left_collision']");o=np.fromstring(left.find('origin').get('xyz'),sep=' ');sz=np.fromstring(left.find('geometry/box').get('size'),sep=' ')
  self.assertAlmostEqual(o[0]-sz[0]/2,-.2)
  self.assertAlmostEqual(sz[1],.4)
  mount_origin=robot.find("joint[@name='sim_hole_plate_joint']/origin")
  mount=np.fromstring(mount_origin.get('xyz'),sep=' ')
  rotation=Rotation.from_euler('xyz',np.fromstring(mount_origin.get('rpy'),sep=' ')).as_matrix()
  np.testing.assert_allclose(rotation[:,2],[0,-1,0],atol=1e-7)  # normal follows finger opening
  self.assertAlmostEqual((mount+rotation@np.array([0,-.2,0]))[2],.1358-.05)
  self.assertEqual(robot.find("joint[@name='sim_hole_plate_joint']/parent").get('link'),'left_gripper_base')
  self.assertEqual(robot.find("joint[@name='sim_right_rod_joint']/parent").get('link'),'right_gripper_base')
  T=np.eye(4)
  for name in ('sim_right_d405_joint','sim_right_d405_optical_joint'):
   o=robot.find(f"joint[@name='{name}']/origin");v=np.fromstring(o.get('xyz'),sep=' ');r=np.fromstring(o.get('rpy'),sep=' ');t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',r).as_matrix();t[:3,3]=v;T=T@t
  np.testing.assert_allclose(T,cal['T_gripper_camera'],atol=1e-12)
  self.assertIsNotNone(robot.find("gazebo[@reference='sim_right_d405_link']/sensor[@type='depth']"))
if __name__=='__main__':unittest.main()
