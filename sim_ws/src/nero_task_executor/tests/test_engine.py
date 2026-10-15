import copy
import json
from pathlib import Path
import pytest
from nero_task_executor.model import validate_task
from nero_task_executor.engine import Engine, MockBackend

EXAMPLE = Path(__file__).resolve().parents[1]/'examples/bottle_transfer.json'


def task():
    return json.loads(EXAMPLE.read_text())


class Clock:
    value = 0.
    def __call__(self):
        return self.value
    def step(self, backend, engine, amount=.1):
        self.value += amount
        backend.poll()
        engine.tick()


def setup(**kwargs):
    clock = Clock()
    backend = MockBackend(clock, **kwargs)
    return clock, backend, Engine(backend, clock=clock)


def test_three_lanes_and_dependencies():
    clock, backend, engine = setup(durations={'b1':2., 'l1':.2, 'r1':.4})
    engine.submit(task())
    engine.tick()
    assert [x[2] for x in backend.events] == ['b1', 'l1', 'r1']
    assert len({x[3] for x in backend.events}) == 1
    for _ in range(100):
        clock.step(backend, engine)
    starts = {x[2]:x[3] for x in backend.events if x[0]=='start'}
    ends = {x[1]:x[2] for x in backend.events if x[0]=='finish'}
    assert starts['r3'] >= ends['b1']
    assert starts['b2'] >= ends['r5']
    assert starts['r7'] >= ends['b2']
    assert starts['b3'] >= ends['r8']
    assert starts['r2'] < ends['b1']  # right arm proceeds while chassis is still moving
    assert engine.snapshot(task()['task_id'])['status'] == 'SUCCEEDED'


@pytest.mark.parametrize('mutation', [
    lambda t: t['base_actions'][0]['wait_for'].append('r3'),  # cycle via implicit sequence edges
    lambda t: t['right_arm_actions'][0]['wait_for'].append('missing'),
    lambda t: t['right_arm_actions'][0].update(action_id='b1'),
    lambda t: t['left_arm_actions'][0]['parameters']['joint_positions_rad'].update(J8=1),
    lambda t: t['right_arm_actions'][0]['parameters'].update(orientation_xyzw=[0,0,0,0]),
    lambda t: t['right_arm_actions'][0]['parameters'].update(position_m=[float('nan'),0,0]),
    lambda t: t['right_arm_actions'][0].update(timeout_s=0),
    lambda t: t['base_actions'][0]['parameters'].update(frame_id='map'),
    lambda t: t.update(version=True),
])
def test_reject_before_any_dispatch(mutation):
    _, backend, engine = setup()
    t = task()
    mutation(t)
    with pytest.raises(ValueError):
        engine.submit(t)
    assert backend.events == []


def test_duplicate_keys_and_null():
    with pytest.raises(ValueError):
        validate_task('{"version":1,"version":2}')
    t = task()
    t['left_arm_actions'][0]['parameters']['joint_positions_rad']['J1'] = None
    with pytest.raises(ValueError):
        validate_task(t)


def test_failure_blocks_all_future_actions():
    clock, backend, engine = setup(fail={'r4'})
    engine.submit(task())
    for _ in range(100):
        clock.step(backend, engine)
    assert engine.snapshot(task()['task_id'])['status'] == 'FAILED'
    assert not {'b2','b3','r5','r7'} & {x[2] for x in backend.events if x[0]=='start'}


def test_timeout_cancels_all_running_lanes():
    clock, backend, engine = setup(durations={'r1':100, 'l1':100, 'b1':100})
    t = task()
    t['right_arm_actions'][0]['timeout_s'] = .5
    engine.submit(t)
    engine.tick()
    clock.step(backend, engine, 1.)
    assert engine.snapshot(t['task_id'])['status'] == 'FAILED'
    assert backend.pending == {}


def test_cancel_and_duplicate_and_busy():
    _, backend, engine = setup()
    t = task()
    engine.submit(t)
    engine.tick()
    engine.submit(t)
    assert len(backend.events) == 3
    altered = copy.deepcopy(t)
    altered['task_id'] = 'another'
    with pytest.raises(ValueError, match='BUSY'):
        engine.submit(altered)
    altered['task_id'] = t['task_id']
    altered['left_arm_actions'] = []
    with pytest.raises(ValueError, match='CONFLICT'):
        engine.submit(altered)
    assert engine.cancel(t['task_id'])['status'] == 'CANCELED'


def test_unconfirmed_stop_keeps_robot_owned():
    _, backend, engine = setup()
    backend.cancel = lambda handle, done: done(False, 'Stop unknown')
    t = task()
    engine.submit(t)
    engine.tick()
    assert engine.cancel(t['task_id'])['status'] == 'STOP_UNCONFIRMED'
    t['task_id'] = 'next'
    with pytest.raises(ValueError, match='BUSY'):
        engine.submit(t)


def test_empty_lanes_and_late_result():
    clock, backend, engine = setup()
    t = task()
    t['base_actions'] = []
    t['right_arm_actions'] = []
    engine.submit(t)
    engine.tick()
    old_done = next(iter(backend.pending.values()))[2]
    engine.cancel(t['task_id'])
    t['task_id'] = 'new'
    engine.submit(t)
    engine.tick()
    old_done(True, 'Old feedback')
    assert engine.snapshot('new')['actions']['l1']['status'] == 'RUNNING'
    clock.step(backend, engine, 1)
    assert engine.snapshot('new')['status'] == 'SUCCEEDED'


def test_bottle_places_directly_after_base_arrival():
    """Scenario contract: docking is followed by descent and release, no pose replanning."""
    t = task()
    right = t['right_arm_actions']
    dependent = [i for i, action in enumerate(right) if 'b2' in action['wait_for']]
    assert len(dependent) == 1
    placement = right[dependent[0]:]
    assert [a['type'] for a in placement] == ['move_l', 'gripper_open']
    assert placement[0]['parameters']['position_m'][2] < next(
        a for a in right if a['action_id']=='r5')['parameters']['position_m'][2]


def test_bottle_returns_home_after_release_with_only_left_j2_sixty_degrees():
    import math
    t = task()
    assert t['left_arm_actions'][0]['parameters']['joint_positions_rad'] == {'J2': math.pi/3}
    home = t['base_actions'][-1]
    assert home['action_id'] == 'b3'
    assert home['wait_for'] == ['r8']
    assert home['parameters']['position_xy_m'] == [0, 0]
    assert home['parameters']['yaw_rad'] == 0
    # Translate clear of the destination table before turning in the corridor.
    assert home['parameters']['via'] == [[0, 0, math.pi]]


@pytest.mark.parametrize('kind,parameters', [
    ('base_translate', {'frame_id':'odom','position_xy_m':[.4,0]}),
    ('base_rotate', {'frame_id':'odom','yaw_rad':1.57}),
    ('base_move_and_rotate', {'frame_id':'odom','position_xy_m':[0,0],'yaw_rad':0}),
])
def test_new_base_modes_validate_and_keep_lane_order(kind, parameters):
    clock, backend, engine = setup()
    t = task()
    t['task_id'] = kind
    t['left_arm_actions'] = t['right_arm_actions'] = []
    t['base_actions'] = [{'action_id':'new','type':kind,'parameters':parameters,'wait_for':[],'timeout_s':60}]
    engine.submit(t)
    engine.tick()
    clock.step(backend, engine, 1)
    assert engine.snapshot(kind)['status'] == 'SUCCEEDED'


@pytest.mark.parametrize('kind,parameters', [
    ('base_translate', {'frame_id':'odom','position_xy_m':[0,0],'yaw_rad':1}),
    ('base_rotate', {'frame_id':'odom','yaw_rad':1,'position_xy_m':[0,0]}),
    ('base_move_and_rotate', {'frame_id':'odom','position_xy_m':[0,0]}),
    ('base_move_and_rotate', {'frame_id':'map','position_xy_m':[0,0],'yaw_rad':0}),
    ('base_translate', {'frame_id':'odom','position_xy_m':[0,0],'max_linear_speed_m_s':0}),
    ('base_rotate', {'frame_id':'odom','yaw_rad':1,'max_angular_speed_rad_s':.5}),
    ('base_translate', {'frame_id':'odom','position_xy_m':[0,0],'duration_s':60}),
])
def test_new_base_modes_reject_ambiguous_or_invalid_parameters(kind, parameters):
    t = task()
    t['left_arm_actions'] = t['right_arm_actions'] = []
    t['base_actions'] = [{'action_id':'bad','type':kind,'parameters':parameters,'wait_for':[],'timeout_s':60}]
    with pytest.raises(ValueError):
        validate_task(t)
