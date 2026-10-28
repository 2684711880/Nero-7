import time
from types import SimpleNamespace
from ranger_air_gazebo.lift_control import LiftControl


def controller(height):
    c=LiftControl.__new__(LiftControl)
    c.active=False;c.finish=0;c.future=None;c.checks=[];c.base_stopped=True;c.base_at=time.monotonic()
    c.fresh=lambda:True;c.logs=[]
    c.node=SimpleNamespace(get_logger=lambda:SimpleNamespace(warning=c.logs.append,info=c.logs.append))
    c.validity=SimpleNamespace(service_is_ready=lambda:True)
    c.pub=SimpleNamespace(get_subscription_count=lambda:1)
    c.feedback=SimpleNamespace(name=[c.JOINT,'left_joint2'],position=[height,1.25])
    return c


def test_upper_target_is_clamped_and_path_keeps_arm_angle():
    c=controller(.79);c.request(.05)
    assert c.active and c.target==.8
    assert len(c.checks)==11
    assert all(0<=r.robot_state.joint_state.position[0]<=.8 for r in c.checks)
    assert all(r.robot_state.joint_state.position[1]==1.25 for r in c.checks)


def test_lower_target_is_clamped():
    c=controller(.01);c.request(-.05)
    assert c.active and c.target==0


def test_moving_base_blocks_lift():
    c=controller(.2);c.base_stopped=False;c.request(.05)
    assert not c.active and not c.checks
    assert 'Stop the base' in c.logs[-1]


def test_bad_feedback_does_not_command():
    c=controller(float('nan'));c.request(.05)
    assert not c.active and not c.checks
