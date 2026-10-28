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


@pytest.mark.parametrize('mount',['chassis_up','chest_down','head_tilt'])
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
    assert len(model.boxes)==39


@pytest.mark.parametrize('mount',['chassis_up','chest_down','head_tilt'])
def test_lift_moves_upper_robot_and_only_chest_sensor(mount):
    model=BodyModel(compose(mid360_mount=mount)[0])
    low={k:1.25 if k in ('left_joint2','right_joint2') else 0 for k in model.moving}
    high={**low,'upper_body_lift_joint':.6}
    a,b=model.poses(low),model.poses(high)
    for link in ('dual_nero_base_link','left_gripper_base','right_gripper_base'):
        np.testing.assert_allclose(b[link][:3,3]-a[link][:3,3],[0,0,.6],atol=1e-12)
    np.testing.assert_allclose(b['mid360_link'][:3,3]-a['mid360_link'][:3,3],
                              [0,0,.6 if mount!='chassis_up' else 0],atol=1e-12)
    np.testing.assert_allclose(b['base_link'],a['base_link'])
    if mount=='chest_down':
        np.testing.assert_allclose(a['mid360_link'][:3,3],[.16,0,.76],atol=1e-12)


@pytest.mark.parametrize('degrees',[-90.,0.,30.,45.,60.,90.])
def test_head_pitch_and_lift_transform(degrees):
    model=BodyModel(compose(mid360_mount='head_tilt',mid360_tilt_deg=str(degrees))[0])
    q={k:0. for k in model.moving};q['upper_body_lift_joint']=.4
    pose=model.poses(q)['mid360_link']
    np.testing.assert_allclose(pose[:3,3],[.05,0,1.42],atol=1e-12)
    a=np.deg2rad(degrees)
    np.testing.assert_allclose(pose[:3,:3],[[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]],atol=1e-12)


def test_head_angle_override_precedence_and_invalid_values():
    from mobile_dual_nero.mid360 import mount_pose
    _,rpy=mount_pose('head_tilt')
    assert rpy==[0.,np.pi/4,0.]
    _,rpy=mount_pose('head_tilt',rpy='0 0.2 0',tilt_deg='30')
    assert rpy==[0.,.2,0.]
    for invalid in ['nan','inf','91','-91']:
        with pytest.raises(ValueError):mount_pose('head_tilt',tilt_deg=invalid)
