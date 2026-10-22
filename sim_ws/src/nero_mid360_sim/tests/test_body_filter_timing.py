from collections import deque
from pathlib import Path
import runpy
from types import SimpleNamespace
import time
import numpy as np
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from test_body_geometry import toy

SelfFilter=runpy.run_path(str(Path(__file__).parents[1]/'scripts/body_self_filter'))['SelfFilter']


def make_filter():
    node=SelfFilter.__new__(SelfFilter)
    node.model=toy();node.step=.005
    node.history=deque([(10*10**9,np.array([0.])),(101*10**8,np.array([np.pi/2]))])
    node.pending=deque();node.last_markers=101*10**8
    node.out_rows=[];node.warnings=[]
    node.out=SimpleNamespace(publish=node.out_rows.append)
    node.view=SimpleNamespace(publish=lambda m:None)
    node.stats=SimpleNamespace(publish=lambda m:None)
    node.warn=node.warnings.append
    return node


def message():
    m=CustomMsg();m.header.frame_id='mid360_link';m.header.stamp.sec=10;m.timebase=10**10
    m.lidar_id=7
    m.points=[CustomPoint(x=1.,y=0.,z=0.,offset_time=0,reflectivity=41,tag=16,line=1),
              CustomPoint(x=.2,y=0.,z=0.,offset_time=50000000,reflectivity=42,tag=16,line=2),
              CustomPoint(x=0.,y=1.,z=0.,offset_time=100000000,reflectivity=43,tag=16,line=3)]
    m.point_num=3
    return m


def test_different_joint_poses_within_one_scan_preserve_livox_fields():
    node=make_filter();m=message();node.pending.append((m,time.monotonic()));node.process()
    assert not node.warnings
    out=node.out_rows[0]
    assert out.points==[m.points[1]]
    assert out.header==m.header and out.timebase==m.timebase and out.lidar_id==7
    assert out.point_num==1 and out.points[0].offset_time==50000000


def test_wait_for_joint_feedback_instead_of_forwarding_raw():
    node=make_filter();node.history.pop();node.pending.append((message(),time.monotonic()))
    node.process()
    assert not node.out_rows and len(node.pending)==1
    msg,_=node.pending.pop();node.pending.append((msg,time.monotonic()-3));node.process()
    assert not node.out_rows and not node.pending and node.warnings
