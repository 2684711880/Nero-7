"""Regress stopping at a swing extremum and pressing T again."""
from types import SimpleNamespace
from ranger_air_gazebo.arm_swing import ArmSwing


def swing_at_extremum():
    swing = ArmSwing.__new__(ArmSwing)
    swing.enabled = False
    swing.fresh = lambda: True
    swing.validity = SimpleNamespace(service_is_ready=lambda: True)
    swing.pubs = {s: SimpleNamespace(get_subscription_count=lambda: 1) for s in ('left', 'right')}
    swing.logs = []
    logger = SimpleNamespace(info=swing.logs.append, warning=swing.logs.append)
    swing.node = SimpleNamespace(get_logger=lambda: logger)
    q = {f'{side}_joint{i}': (1.25 if i == 2 else .30 if i == 1 else 0.0)
         for side in ('left', 'right') for i in range(1, 8)}
    swing.feedback = SimpleNamespace(name=list(q), position=list(q.values()))
    return swing


def test_restart_at_commanded_extremum():
    swing = swing_at_extremum()
    swing.toggle()
    assert swing.enabled, swing.logs


def test_resume_path_starts_at_feedback_and_ends_at_fixed_center():
    swing = swing_at_extremum()
    swing.toggle()
    assert swing.enabled
    start, velocity = swing.cycle_state(0.0)
    end, end_velocity = swing.cycle_state(swing.PERIOD)
    for side in ('left', 'right'):
        assert abs(start[f'{side}_joint1']-.30) < 1e-12
        assert abs(end[f'{side}_joint1']) < 1e-12
        assert abs(velocity[f'{side}_joint1']) < 1e-12
        assert abs(end_velocity[f'{side}_joint1']) < 1e-12
    for i in range(101):
        q, _ = swing.cycle_state(swing.PERIOD*i/100)
        assert max(abs(q[f'{s}_joint1']) for s in ('left', 'right')) <= .30+1e-12


def test_rejection_identifies_joint():
    swing = swing_at_extremum()
    swing.feedback.position[swing.feedback.name.index('right_joint4')] = 1.0
    swing.toggle()
    assert not swing.enabled
    assert 'right_joint4' in swing.logs[-1]


def test_swing_accepts_new_tucked_j2_pose():
    swing = swing_at_extremum()
    for side in ('left', 'right'):
        swing.feedback.position[swing.feedback.name.index(f'{side}_joint2')] = 1.45
    swing.toggle()
    assert swing.enabled, swing.logs
