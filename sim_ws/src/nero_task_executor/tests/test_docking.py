from types import SimpleNamespace as Obj
from nero_task_executor.sim_backend import SimBackend


def backend():
    b = SimBackend.__new__(SimBackend)
    b.arrival_target = None
    b.arrived = False
    b.commands = []
    b.velocity = Obj(publish=b.commands.append)
    b.pose = [0., 0., 0.]
    b.base_pose = lambda: b.pose
    b.stopped_base = lambda: True
    return b


def test_boundary_noise_does_not_restart_docking():
    b = backend()
    target = [0., 0., 0.]
    b.pose[0] = .014
    assert not b.drive(target)  # must first enter the tighter tolerance
    b.pose[0] = .0119
    assert b.drive(target)
    for noisy_distance in (.0149, .0151, .0148, .0152):
        b.pose[0] = noisy_distance
        assert b.drive(target)
        assert b.commands[-1].linear.x == 0.
    b.pose[0] = .026
    assert not b.drive(target)  # real drift exits the wider tolerance


def test_next_waypoint_requires_its_own_arrival():
    b = backend()
    assert b.drive([0., 0., 0.])
    assert not b.drive([0., .8, 0.])
    assert not b.arrived


def test_arrival_also_requires_stopped_velocity():
    b = backend()
    b.stopped_base = lambda: False
    assert not b.drive([0., 0., 0.])
    assert b.commands[-1].linear.x == 0.


def test_turn_command_does_not_fall_into_observed_stall_range():
    """Replay live failure: yaw=3.04507, target=pi, old command=0.0772 rad/s."""
    import math
    b = backend()
    b.pose[2] = 3.0450695
    assert not b.drive([0., 0., math.pi])
    assert .12 <= b.commands[-1].angular.z <= .18


def test_small_remaining_turn_still_has_drive_until_arrived():
    b = backend()
    b.pose[2] = -.016
    assert not b.drive([0., 0., 0.])
    assert b.commands[-1].angular.z >= .12
    b.pose[2] = -.014
    assert b.drive([0., 0., 0.])
    assert b.commands[-1].angular.z == 0.


def test_linear_docking_does_not_fall_into_observed_stall_range():
    """New J2 posture stalled 27 mm short with only a 0.0163 m/s command."""
    import math
    b = backend()
    b.pose = [.00028, -.7727855, .000481]
    assert not b.drive([0., -.8, 0.])
    velocity = b.commands[-1].linear
    assert .04-1e-9 <= math.hypot(velocity.x, velocity.y) <= .1+1e-9
