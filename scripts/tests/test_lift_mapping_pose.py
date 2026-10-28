"""Moving sensor-to-base extrinsics must not become fictitious base vertical motion."""
import runpy
from pathlib import Path
import numpy as np
module=runpy.run_path(str(Path(__file__).parents[2]/'sim_ws/src/nero_mid360_sim/scripts/mapping_robot_pose'))


def test_interpolate_lift_at_sensor_timestamp():
    h=module['LiftHistory']()
    h.add(1_000_000_000,0.);h.add(1_100_000_000,.1)
    assert h.sample(1_050_000_000)==.05
    assert h.sample(1_200_000_000) is None
    h.add(1,0.)
    assert h.sample(1_050_000_000) is None


def test_chest_sensor_rises_while_displayed_base_stays_at_ground():
    for height in [0.,.05,.4,.8]:
        p,q=module['base_pose']((.16,0,.76+height),(1,0,0,0),(.16,0,.76+height),(np.pi,0,0))
        np.testing.assert_allclose(p,[0,0,0],atol=1e-12)
        np.testing.assert_allclose(q,[0,0,0,1],atol=1e-12)


def test_tilted_head_sensor_lift_does_not_move_displayed_base():
    angle=np.pi/4
    orientation=(0,np.sin(angle/2),0,np.cos(angle/2))
    for height in [0.,.2,.8]:
        p,q=module['base_pose']((.05,0,1.02+height),orientation,(.05,0,1.02+height),(0,angle,0))
        np.testing.assert_allclose(p,[0,0,0],atol=1e-12)
        np.testing.assert_allclose(q,[0,0,0,1],atol=1e-12)
