"""Strict, ROS-independent version-1 task validation. No motion on invalid input."""
import copy
import json
import math

LANES = ('base', 'left_arm', 'right_arm')
POSE_KEYS = {'frame_id', 'tcp_id', 'position_m', 'orientation_xyzw'}
ARM_TYPES = {'move_j', 'move_j_partial', 'move_p', 'move_l', 'move_c',
             'gripper_open', 'gripper_grasp'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fields(obj, required, optional=()):
    require(isinstance(obj, dict), 'Expected an object')
    require(set(required) <= obj.keys(), 'Missing fields: '+str(set(required)-obj.keys()))
    require(obj.keys() <= set(required) | set(optional),
            'Unknown fields: '+str(obj.keys()-set(required)-set(optional)))


def number(value, positive=False):
    require(type(value) in (float, int) and math.isfinite(value), 'Expected a finite number')
    require(not positive or value > 0, 'Expected a positive number')


def vector(value, length):
    require(isinstance(value, list) and len(value) == length, f'Expected {length} numbers')
    for v in value:
        number(v)


def name(value):
    require(isinstance(value, str) and 0 < len(value) <= 128, 'Invalid name')


def pose(p):
    fields(p, POSE_KEYS, {'touch_object'})
    if 'touch_object' in p:
        require(p['touch_object'] == 'water_bottle', 'Only water_bottle contact is supported')
    name(p['frame_id'])
    require(p['tcp_id'] == 'default_tcp', 'Only default_tcp is supported in v1')
    vector(p['position_m'], 3)
    vector(p['orientation_xyzw'], 4)
    require(abs(sum(x*x for x in p['orientation_xyzw'])-1) < 1e-6,
            'Quaternion must be normalized')


def validate_action(a, lane):
    fields(a, {'action_id', 'type', 'parameters', 'wait_for', 'timeout_s'})
    name(a['action_id'])
    number(a['timeout_s'], positive=True)
    require(a['timeout_s'] <= 86400, 'Action timeout exceeds one day')
    require(isinstance(a['wait_for'], list), 'wait_for must be an array')
    for dep in a['wait_for']:
        name(dep)
    require(len(set(a['wait_for'])) == len(a['wait_for']), 'Duplicate dependency')
    kind, p = a['type'], a['parameters']
    require(isinstance(kind, str), 'Invalid action type')
    require(kind in ({'base_move_to'} if lane == 'base' else ARM_TYPES),
            f'Unsupported {lane} action: {kind}')
    if kind == 'base_move_to':
        fields(p, {'frame_id', 'position_xy_m', 'yaw_rad'}, {'via'})
        if 'via' in p:
            require(isinstance(p['via'], list) and len(p['via']) <= 100, 'Invalid route')
            for point in p['via']:
                vector(point, 3)
        require(p['frame_id'] == 'odom', 'v1 base controller supports odom only')
        vector(p['position_xy_m'], 2)
        number(p['yaw_rad'])
    elif kind == 'move_j':
        fields(p, {'joint_positions_rad'})
        vector(p['joint_positions_rad'], 7)
    elif kind == 'move_j_partial':
        fields(p, {'joint_positions_rad'})
        joints = p['joint_positions_rad']
        require(isinstance(joints, dict) and len(joints) > 0, 'Empty joint patch')
        require(joints.keys() <= {f'J{i}' for i in range(1, 8)}, 'Unknown joint')
        for value in joints.values():
            number(value)
    elif kind in ('move_p', 'move_l'):
        pose(p)
        require('touch_object' not in p or lane == 'right_arm', 'Contact profile belongs to right arm')
    elif kind == 'move_c':
        fields(p, {'poses'})
        require(isinstance(p['poses'], list) and len(p['poses']) == 3, 'Arc needs three poses')
        for item in p['poses']:
            pose(item)
            require('touch_object' not in item, 'Arc contact profiles are not supported')
        require(len({x['frame_id'] for x in p['poses']}) == 1, 'Arc frames must match')
        a0, b, c = [x['position_m'] for x in p['poses']]
        u, v = [b[i]-a0[i] for i in range(3)], [c[i]-a0[i] for i in range(3)]
        cross = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
        scale = math.sqrt(sum(x*x for x in u)*sum(x*x for x in v))
        require(scale > 1e-12 and math.sqrt(sum(x*x for x in cross)) / scale > 1e-6,
                'Degenerate arc')
    elif kind == 'gripper_open':
        fields(p, {'opening_m'})
        number(p['opening_m'])
        require(0 <= p['opening_m'] <= .1, 'Gripper opening outside 0..0.1 m')
    else:
        fields(p, {'grasp_profile'})
        require(lane == 'right_arm' and p['grasp_profile'] == 'water_bottle',
                'Only the right-arm water_bottle simulation grasp is configured')


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'Duplicate JSON key: '+key)
        result[key] = value
    return result


def validate_task(raw):
    if isinstance(raw, str):
        require(len(raw.encode('utf-8')) <= 1024*1024, 'Task exceeds 1 MiB')
        raw = json.loads(raw, object_pairs_hook=_unique_object)
    task = copy.deepcopy(raw)
    fields(task, {'version', 'task_id'} | {x+'_actions' for x in LANES})
    require(type(task['version']) is int and task['version'] == 1, 'Unsupported version')
    name(task['task_id'])
    actions, dependencies = {}, {}
    for lane in LANES:
        seq = task[lane+'_actions']
        require(isinstance(seq, list), 'Actions must be arrays')
        previous = None
        for action in seq:
            validate_action(action, lane)
            key = action['action_id']
            require(key not in actions, 'Duplicate action_id: '+key)
            actions[key] = action
            dependencies[key] = set(action['wait_for']) | ({previous} if previous else set())
            previous = key
    require(0 < len(actions) <= 1000, 'Task must contain 1..1000 actions')
    for key, deps in dependencies.items():
        require(deps <= actions.keys(), 'Unknown dependency in '+key)
    # Iterative topological check includes implicit per-lane edges.
    remaining = {key: set(value) for key, value in dependencies.items()}
    while remaining:
        ready = {key for key, deps in remaining.items() if not deps}
        require(bool(ready), 'Dependency cycle (including sequence order)')
        remaining = {key: deps-ready for key, deps in remaining.items() if key not in ready}
    return task
