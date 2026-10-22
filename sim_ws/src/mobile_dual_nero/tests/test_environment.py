"""Check task geometry and Gazebo/MoveIt parity before launching physics."""
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from mobile_dual_nero.convenience_store import convenience_store_objects, RACK_X, RACK_WIDTH
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


    def test_store_ceiling_toggle_removes_physics_and_planning_geometry(self):
        objects = convenience_store_objects(ceiling=False)
        template = Path(get_package_share_directory('ranger_air_gazebo')) / 'worlds/control.world'
        world = ET.fromstring(world_xml(template, objects, scene='convenience_store')).find('world')
        self.assertIsNone(world.find("model[@name='store_ceiling']"))
        self.assertNotIn('store_ceiling', {row.split()[0] for row in collision_rows(objects)})

    def test_store_collision_parity_and_start_clearance(self):
        objects = convenience_store_objects()
        template = Path(get_package_share_directory('ranger_air_gazebo')) / 'worlds/control.world'
        world = ET.fromstring(world_xml(template, objects, scene='convenience_store')).find('world')
        self.assertEqual(world.get('name'), 'convenience_store')
        self.assertEqual(len(objects), len({o['name'] for o in objects}))
        rows = {row.split()[0] for row in collision_rows(objects)}
        for obj in objects:
            model = world.find(f"model[@name='{obj['name']}']")
            collision = model.find('link/collision')
            self.assertEqual(collision is not None, obj.get('collision', True))
            self.assertEqual(obj['name'] in rows, collision is not None)
            if collision is None:
                continue
            self.assertEqual(list(map(float, model.findtext('pose').split()[:3])), obj['xyz'])
            geom = collision.find('geometry')
            if obj['kind'] == 'box':
                size = list(map(float, geom.findtext('box/size').split()))
                dx = max(abs(obj['xyz'][0]) - size[0]/2, 0)
                dy = max(abs(obj['xyz'][1]) - size[1]/2, 0)
            else:
                size = [float(geom.findtext('cylinder/length')), float(geom.findtext('cylinder/radius'))]
                dx = max(abs(obj['xyz'][0]) - size[1], 0)
                dy = max(abs(obj['xyz'][1]) - size[1], 0)
            self.assertEqual(size, obj['size'])
            if obj['kind'] == 'box' and obj['xyz'][2] - obj['size'][2]/2 > 2.2:
                continue  # ceiling is above the complete initial robot envelope
            self.assertGreater((dx*dx + dy*dy)**.5, .70, obj['name'])
        # Gondola-to-gondola shelf edges define the narrowest regular aisle.
        self.assertAlmostEqual(RACK_X[1] - RACK_X[0] - RACK_WIDTH, 1.65)
        self.assertNotIn('water_bottle', rows)
        roof = world.find("model[@name='store_ceiling']/link")
        self.assertIsNotNone(roof.find('collision'))
        self.assertIsNone(roof.find('visual'))


if __name__ == '__main__':
    unittest.main()
