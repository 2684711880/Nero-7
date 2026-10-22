import numpy as np
import pytest
from nero_mid360_sim.body_geometry import BodyModel
from mobile_dual_nero.model import compose


def toy(padding=0):
    return BodyModel('''<robot name="test"><link name="base_link"/>
      <link name="mid360_link"/><joint name="mount" type="fixed"><parent link="base_link"/><child link="mid360_link"/></joint>
      <link name="arm"><collision><origin xyz="1 0 0"/><geometry><box size="0.4 0.2 0.2"/></geometry></collision></link>
      <joint name="hinge" type="revolute"><parent link="base_link"/><child link="arm"/><axis xyz="0 0 1"/></joint></robot>''',padding)


def test_moving_body_and_nearby_environment():
    m=toy()
    points=np.array([[1.,0,0],[0,1.,0],[1.,.2,0],[.2,0,0]])
    assert m.mask(points,{'hinge':0}).tolist()==[True,False,False,False]
    assert m.mask(points,{'hinge':np.pi/2}).tolist()==[False,True,False,False]


def test_padding_margin():
    p=np.array([[1.,.12,0],[1.,.16,0]])
    assert toy(.025).mask(p,{'hinge':0}).tolist()==[True,False]


@pytest.mark.parametrize('mount',['chassis_up','chest_down'])
def test_full_robot_boxes_cover_their_surfaces(mount):
    model=BodyModel(compose(mid360_mount=mount)[0])
    q={k:1.25 if k in ('left_joint2','right_joint2') else 0 for k in model.moving}
    poses=model.poses(q);sensor=np.linalg.inv(poses['mid360_link'])
    points=[]
    for link,origin,center,half in model.boxes:
        T=sensor@poses[link]@origin
        for axis in range(3):
            for sign in [-1,1]:
                local=center.copy();local[axis]+=sign*(half[axis]-.001)
                points.append(T[:3,:3]@local+T[:3,3])
    assert model.mask(np.array(points),q).all()
    assert len(model.boxes)==38
