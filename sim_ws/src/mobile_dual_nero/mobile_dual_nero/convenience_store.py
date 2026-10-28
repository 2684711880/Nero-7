"""Editable convenience-store geometry, metres in world/odom.

No external model downloads. Goods and fixtures have physical collision shapes;
price strips, floor markings and product labels are visual decorations only.
The ceiling collision remains active; its visual is hidden for inspection.
"""

STORE_SIZE = (12.0, 10.0)
RACK_Y = 2.3
RACK_LENGTH = 3.0
RACK_WIDTH = .85
RACK_HEIGHT = 1.80
AISLE_WIDTH = 1.20
RACK_X = (-(RACK_WIDTH + AISLE_WIDTH), 0.0, RACK_WIDTH + AISLE_WIDTH)
SHELF_TOPS = (.20, .60, 1.00, 1.45)


def convenience_store_objects(ceiling=True):
    objects = []
    palette = ['.86 .20 .15 1', '.96 .65 .08 1', '.12 .55 .32 1',
               '.16 .45 .78 1', '.65 .28 .57 1', '.87 .79 .57 1']
    white = '.88 .90 .86 1'
    steel = '.18 .23 .26 1'
    teal = '.03 .43 .39 1'

    def add(name, size, xyz, color, kind='box', collision=True):
        item = dict(name=name, kind=kind, size=list(size), xyz=list(xyz), color=color)
        if not collision:
            item['collision'] = False
        objects.append(item)
        return item

    # Three full walls, and a front wall with a 2 m entrance opening.
    for name, size, xyz in [
        ('north', (12.2, .18, 3.1), (0, 5.09, 1.55)),
        ('west', (.18, 10, 3.1), (-6.09, 0, 1.55)),
        ('east', (.18, 10, 3.1), (6.09, 0, 1.55)),
        ('front_left', (5, .18, 1.05), (-3.5, -5.09, .525)),
        ('front_right', (5, .18, 1.05), (3.5, -5.09, .525)),
        ('entry_header', (12.2, .18, .35), (0, -5.09, 2.925)),
        ('entry_post_left', (.08, .18, 2.75), (-1.04, -5.09, 1.375)),
        ('entry_post_right', (.08, .18, 2.75), (1.04, -5.09, 1.375)),
    ]:
        add('store_wall_' + name, size, xyz, white)
    # Hidden ceiling visual keeps the camera view open while indoor rays hit the roof.
    if ceiling:
        roof = add('store_ceiling', (12.0, 10.0, .12), (0, 0, 3.16), white)
        roof['visible'] = False
    # Upper front panels use opaque collision returns; translucent visuals are schematic.
    for side, x in [('left', -3.5), ('right', 3.5)]:
        pane = add('store_front_upper_' + side, (5.0, .18, 1.70), (x, -5.09, 1.90), '.65 .78 .80 1')
        pane['transparency'] = .82
    # Familiar green/orange convenience-store fascia; no fictitious glass physics.
    for i, (z, color) in enumerate([(2.83, teal), (2.96, '.96 .54 .10 1')]):
        add(f'front_fascia_{i}', (12.2, .025, .10), (0, -5.195, z), color, collision=False)
        add(f'back_fascia_{i}', (11.98, .025, .10), (0, 4.985, z), color, collision=False)
    # Tile grout is purely visual, on the existing z=0 ground plane.
    for i in range(25):
        add(f'tile_x_{i}', (.012, 10, .001), (-6 + i*.5, 0, .001), '.63 .65 .63 1', collision=False)
    for i in range(21):
        add(f'tile_y_{i}', (12, .012, .001), (0, -5 + i*.5, .001), '.63 .65 .63 1', collision=False)

    # Three double-sided gondolas. Shelf and price-strip outer edges leave 1.20 m clear aisles.
    for rack, x in enumerate(RACK_X):
        prefix = f'gondola_{rack}'
        add(prefix + '_plinth', (RACK_WIDTH, RACK_LENGTH, .16), (x, RACK_Y, .08), teal)
        add(prefix + '_divider', (.045, RACK_LENGTH, RACK_HEIGHT-.16), (x, RACK_Y, (RACK_HEIGHT+.16)/2), white)
        for end, y in enumerate((RACK_Y - RACK_LENGTH/2, RACK_Y + RACK_LENGTH/2)):
            add(prefix + f'_end_{end}', (.08, .055, RACK_HEIGHT), (x, y, RACK_HEIGHT/2), steel)
        for tier, top in enumerate(SHELF_TOPS):
            add(prefix + f'_shelf_{tier}', (RACK_WIDTH, RACK_LENGTH, .035),
                (x, RACK_Y, top - .0175), white)
            for side in (-1, 1):
                add(prefix + f'_price_{tier}_{side}', (.016, RACK_LENGTH, .055),
                    (x + side*(RACK_WIDTH/2-.008), RACK_Y, top - .025), '.95 .68 .08 1', collision=False)
                for slot in range(5):
                    y = RACK_Y - 1.2 + slot*.6
                    color = palette[(slot + tier + rack) % len(palette)]
                    name = prefix + f'_goods_{tier}_{side}_{slot}'
                    px = x + side*.25
                    if (tier + rack) % 2:
                        height = .22 + .025*(slot % 3)
                        add(name, (height, .065), (px, y, top + height/2), color, kind='cylinder')
                        add(name + '_label', (.135, .025, .08),
                            (px, y-.062, top+.11), white, collision=False)
                    else:
                        height = .23 + .025*(slot % 3)
                        add(name, (.22, .30, height), (px, y, top + height/2), color)
                        add(name + '_label', (.012, .20, .08),
                            (px + side*.116, y, top+.12), white, collision=False)
        # Colored category header above the merchandise.
        add(prefix + '_header', (.10, 2.95, .18), (x, RACK_Y, RACK_HEIGHT-.09), palette[rack])

    # West wall merchandise: low shelf + tall back panel with stocked tiers.
    add('wall_shelf_back', (.08, 5.0, RACK_HEIGHT), (-5.88, 1.75, RACK_HEIGHT/2), teal)
    for tier, top in enumerate((.22, .62, 1.02, 1.42)):
        add(f'wall_shelf_{tier}', (.60, 5, .045), (-5.56, 1.75, top-.0225), white)
        for slot in range(7):
            add(f'wall_goods_{tier}_{slot}', (.25, .38, .29),
                (-5.55, -.35 + slot*.70, top+.145), palette[(slot+tier) % 6])

    # Three closed refrigerator cabinets, simplified opaque door panels.
    # Glass transmission/multipath cannot be inferred from this geometry.
    for i, y in enumerate((.0, 1.55, 3.1)):
        add(f'fridge_{i}_body', (.78, 1.4, 2.15), (5.50, y, 1.075), steel)
        add(f'fridge_{i}_door', (.035, 1.28, 1.74), (5.09, y, 1.12), '.37 .65 .70 1')
        add(f'fridge_{i}_handle', (.07, .045, .66), (5.03, y-.49, 1.12), white)
        add(f'fridge_{i}_header', (.04, 1.27, .16), (5.08, y, 2.02), teal, collision=False)
        for row in range(3):
            add(f'fridge_{i}_tray_{row}', (.015, 1.17, .025),
                (5.064, y, .55+row*.46), white, collision=False)
            for slot in range(5):
                add(f'fridge_{i}_drink_{row}_{slot}', (.018, .13, .26),
                    (5.05, y-.45+slot*.225, .70+row*.46), palette[(slot+row+i) % 6], collision=False)

    # Checkout and back counter in the southwest; low island in southeast.
    add('checkout_base', (2.6, .85, .91), (-3.55, -3.35, .455), teal)
    add('checkout_top', (2.75, 1.0, .06), (-3.55, -3.35, .94), '.22 .24 .23 1')
    add('register_stand', (.12, .12, .23), (-3.10, -3.35, 1.085), steel)
    add('register_display', (.40, .10, .28), (-3.10, -3.35, 1.33), steel)
    add('register_screen', (.34, .012, .21), (-3.10, -3.407, 1.33), '.13 .65 .70 1', collision=False)
    add('checkout_scanner', (.24, .21, .09), (-3.85, -3.35, 1.015), white)
    add('back_counter', (.60, 1.7, .85), (-5.55, -3.1, .425), white)
    add('coffee_machine', (.42, .47, .54), (-5.52, -3.15, 1.12), steel)
    add('coffee_panel', (.025, .30, .20), (-5.29, -3.15, 1.18), '.62 .32 .16 1', collision=False)
    add('promo_island', (1.45, 1.10, .72), (3.2, -2.8, .36), '.80 .44 .19 1')
    add('promo_top', (1.55, 1.20, .045), (3.2, -2.8, .7425), white)
    for i in range(6):
        add(f'promo_carton_{i}', (.32, .35, .24),
            (2.72+(i % 3)*.46, -3.06+(i//3)*.5, .885), palette[i])
    # Unequal end landmarks break the otherwise repetitive shelf geometry.
    add('stock_carton_low', (.55, .45, .32), (-4.4, 4.48, .16), '.63 .44 .25 1')
    add('stock_carton_high', (.42, .38, .48), (-3.72, 4.48, .24), '.70 .51 .31 1')
    return objects
