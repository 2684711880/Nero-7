"""Check task geometry and Gazebo/MoveIt parity before launching physics."""
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from mobile_dual_nero.environment import warehouse_objects, collision_rows, world_xml


class EnvironmentTest(unittest.TestCase):
    def test_world_matches_collision_geometry(self):
        objects = warehouse_objects()
        template = Path(get_package_share_directory('ranger_air_gazebo')) / 'worlds/control.world'
        world = ET.fromstring(world_xml(template, objects)).find('world')
        names = [o['name'] for o in objects]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(len(collision_rows(objects)), len(objects))
        for obj in objects:
            model = world.find(f"model[@name='{obj['name']}']")
            self.assertEqual(list(map(float, model.findtext('pose').split()[:3])), obj['xyz'])
            geometry = model.find('link/collision/geometry')
            if obj['kind'] == 'box':
                size = list(map(float, geometry.findtext('box/size').split()))
            else:
                size = [float(geometry.findtext('cylinder/length')), float(geometry.findtext('cylinder/radius'))]
            self.assertEqual(size, obj['size'])
        bottle = next(o for o in objects if o['name'] == 'water_bottle')
        table = next(o for o in objects if o['name'] == 'source_table_top')
        gap = bottle['xyz'][2] - bottle['size'][0]/2 - (table['xyz'][2]+table['size'][2]/2)
        self.assertAlmostEqual(gap, .001)
        self.assertEqual(world.findtext("model[@name='water_bottle']/static"), 'false')
        self.assertEqual(len([n for n in names if '_shelf_' in n]), 8)
        # Table inner edges leave a 2.8 m central corridor.
        source = next(o for o in objects if o['name'] == 'source_table_top')
        destination = next(o for o in objects if o['name'] == 'destination_table_top')
        gap = destination['xyz'][1] - destination['size'][1]/2 - (source['xyz'][1] + source['size'][1]/2)
        self.assertAlmostEqual(gap, 2.8)


if __name__ == '__main__':
    unittest.main()
