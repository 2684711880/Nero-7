"""One geometry definition shared by Gazebo and MoveIt (metres, world/odom)."""
import xml.etree.ElementTree as ET


# 修改 TABLE_HEIGHT 即可同时调整两张桌子和桌上水瓶，单位：米。
TABLE_HEIGHT = 0.6  # 桌面上表面距地面的高度
TABLE_TOP_THICKNESS = 0.05
BOTTLE_HEIGHT = 0.24


COLORS = {
    'frame': '.12 .18 .23 1', 'shelf': '.25 .40 .51 1',
    'wood': '.64 .43 .24 1', 'leg': '.25 .28 .30 1',
    'cargo': '.70 .49 .28 1', 'water': '.22 .65 .85 1',
}


def warehouse_objects():
    objects = []

    def box(name, size, xyz, color):
        objects.append(dict(name=name, kind='box', size=size, xyz=xyz, color=COLORS[color]))

    # Geometry positions refer to centres; legs start at ground z=0.
    leg_height = TABLE_HEIGHT - TABLE_TOP_THICKNESS
    if leg_height <= 0:
        raise ValueError('TABLE_HEIGHT must exceed TABLE_TOP_THICKNESS')
    # Two workstations flank the starting pose.
    for name, y in [('source_table', -1.75), ('destination_table', 1.75)]:
        box(name + '_top', [1.2, .7, TABLE_TOP_THICKNESS],
            [0, y, TABLE_HEIGHT - TABLE_TOP_THICKNESS / 2], 'wood')
        for i, (x, dy) in enumerate([(-.52, -.27), (-.52, .27), (.52, -.27), (.52, .27)]):
            box(name + f'_leg_{i}', [.06, .06, leg_height],
                [x, y + dy, leg_height / 2], 'leg')
    # Two separate racks face the central working area, with four open tiers.
    for rack, y in [('rack_a', -1.15), ('rack_b', 1.15)]:
        for i, (dx, dy) in enumerate([(-.23, -.95), (-.23, .95), (.23, -.95), (.23, .95)]):
            box(rack + f'_post_{i}', [.04, .04, 1.85], [2.5 + dx, y + dy, .925], 'frame')
        for tier, top in enumerate([.30, .75, 1.20, 1.65]):
            box(rack + f'_shelf_{tier}', [.5, 2., .04], [2.5, y, top - .02], 'shelf')
        # Back brace, open front for placing goods.
        box(rack + '_back_brace', [.035, 1.94, .08], [2.745, y, 1.76], 'frame')
        for i, (dy, top, height) in enumerate([(-.55, .30, .22), (.45, .75, .18), (-.45, 1.20, .23)]):
            box(rack + f'_cargo_{i}', [.26, .32, height], [2.5, y + dy, top + height / 2], 'cargo')
    # Closed 500 mL bottle: simplified rigid collision cylinder, 0.52 kg.
    objects.append(dict(name='water_bottle', kind='cylinder', size=[BOTTLE_HEIGHT, .034],
                        xyz=[0, -1.60, TABLE_HEIGHT + BOTTLE_HEIGHT / 2 + .001], color=COLORS['water'], mass=.52))
    return objects


def collision_rows(objects):
    """String rows preserve a simple typed ROS parameter interface to C++."""
    return [' '.join(map(str, [o['name'], o['kind'], *o['size'], *o['xyz']])) for o in objects]


def world_xml(template, objects):
    root = ET.parse(template).getroot()
    world = root.find('world')
    world.set('name', 'sorting_warehouse')
    world.find('gui/camera/pose').text = '-5 -6 4.6 0 .48 .75'
    world.find('scene/background').text = '.83 .88 .92 1'
    world.find('.//visual[@name="ground_visual"]/material/ambient').text = '.48 .51 .54 1'
    world.find('.//visual[@name="ground_visual"]/material/diffuse').text = '.48 .51 .54 1'
    for obj in objects:
        model = ET.SubElement(world, 'model', name=obj['name'])
        ET.SubElement(model, 'static').text = 'false' if 'mass' in obj else 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, [*obj['xyz'], 0, 0, 0]))
        link = ET.SubElement(model, 'link', name='body')
        if 'mass' in obj:
            mass = obj['mass']
            height, radius = obj['size']
            inertial = ET.SubElement(link, 'inertial')
            ET.SubElement(inertial, 'mass').text = str(mass)
            inertia = ET.SubElement(inertial, 'inertia')
            for key, value in dict(ixx=mass*(3*radius**2+height**2)/12,
                                   iyy=mass*(3*radius**2+height**2)/12,
                                   izz=mass*radius**2/2, ixy=0, ixz=0, iyz=0).items():
                ET.SubElement(inertia, key).text = str(value)
        for tag in ['collision', 'visual']:
            element = ET.SubElement(link, tag, name='body_' + tag)
            geometry = ET.SubElement(element, 'geometry')
            primitive = ET.SubElement(geometry, obj['kind'])
            if obj['kind'] == 'box':
                ET.SubElement(primitive, 'size').text = ' '.join(map(str, obj['size']))
            else:
                ET.SubElement(primitive, 'length').text = str(obj['size'][0])
                ET.SubElement(primitive, 'radius').text = str(obj['size'][1])
            if tag == 'visual':
                material = ET.SubElement(element, 'material')
                for key in ['ambient', 'diffuse']:
                    ET.SubElement(material, key).text = obj['color']
            else:
                surface = ET.SubElement(element, 'surface')
                ode = ET.SubElement(ET.SubElement(surface, 'friction'), 'ode')
                ET.SubElement(ode, 'mu').text = '.8'
                ET.SubElement(ode, 'mu2').text = '.8'
        if obj['name'] == 'water_bottle':
            # Cap and label remain visual parts of the same rigid body.
            for name, radius, length, z, color in [('cap', .024, .025, .1075, '.05 .22 .65 1'),
                                                  ('label', .0345, .07, -.01, '.92 .96 1 1')]:
                visual = ET.SubElement(link, 'visual', name=name)
                ET.SubElement(visual, 'pose').text = f'0 0 {z} 0 0 0'
                cylinder = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
                ET.SubElement(cylinder, 'radius').text = str(radius)
                ET.SubElement(cylinder, 'length').text = str(length)
                material = ET.SubElement(visual, 'material')
                ET.SubElement(material, 'ambient').text = color
                ET.SubElement(material, 'diffuse').text = color
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode') + '\n'
