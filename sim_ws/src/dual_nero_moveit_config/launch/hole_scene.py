"""Simulation-only plate, tool and depth sensor; never opens hardware."""
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def sub(p,tag,**attrs): return ET.SubElement(p,tag,**attrs)
def origin(p,xyz,rpy=(0,0,0)):
    sub(p,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
def fixed(robot,name,parent,child,xyz,rpy=(0,0,0)):
    j=sub(robot,'joint',name=name,type='fixed');sub(j,'parent',link=parent);sub(j,'child',link=child);origin(j,xyz,rpy)
def inertial(link,mass=.01):
    i=sub(link,'inertial');sub(i,'mass',value=str(mass));sub(i,'inertia',ixx='0.0001',ixy='0',ixz='0',iyy='0.0001',iyz='0',izz='0.0001')
def box(link,name,xyz,rpy,size,visual=True,collision=True):
    for typ in (['visual'] if visual else [])+(['collision'] if collision else []):
        b=sub(link,typ,name=name+'_'+typ);origin(b,xyz,rpy);g=sub(b,'geometry');sub(g,'box',size=' '.join(map(str,size)))
        if typ=='visual':sub(sub(b,'material',name='hole_plate_material'),'color',rgba='0.75 0.78 0.8 1')

def add_hole_scene(robot,semantic,calibration):
    T=np.asarray(calibration['T_gripper_camera'],dtype=float)
    if T.shape!=(4,4):raise ValueError('Invalid hand-eye matrix')
    # Left fingers hold the plate edge: this attachment is a simulation constraint,
    # not a physics claim about grip friction. Board plane is the local XY plane.
    plate=sub(robot,'link',name='sim_hole_plate');inertial(plate,.08)
    # Fingers extend toward -Z and open along Y in the gripper frame.
    # Rotate the plate normal onto Y and insert its near edge 50 mm.
    fixed(robot,'sim_hole_plate_joint','left_gripper_base','sim_hole_plate',[0,0,.2858],[math.pi/2,0,0])
    # A 400 mm square with a 90 mm central hole. Four outer strips form the
    # square; small tangent sectors fill the region around the aperture.
    # The near edge is y=-.20 in board coordinates.
    for name,xyz,size in [
        ('left',[-.125,0,0],[.15,.40,.008]),
        ('right',[.125,0,0],[.15,.40,.008]),
        ('top',[0,.125,0],[.10,.15,.008]),
        ('bottom',[0,-.125,0],[.10,.15,.008]),
    ]:box(plate,name,xyz,[0,0,0],size)
    n=32; inner=.045; outer=math.sqrt(2)*.05; width=2*outer*math.tan(math.pi/n)
    for i in range(n):
        a=2*math.pi*i/n; rad=(inner+outer)/2
        box(plate,f'sector_{i}',[rad*math.cos(a),rad*math.sin(a),0],[0,0,a],[outer-inner,width,.008])
    # Rod axis is local gripper +Z; its origin is an explicit model assumption.
    rod=sub(robot,'link',name='sim_right_rod');inertial(rod,.02)
    fixed(robot,'sim_right_rod_joint','right_gripper_base','sim_right_rod',[0,0,.14])
    for typ in ('visual','collision'):
        e=sub(rod,typ,name='rod_'+typ);origin(e,[0,0,.02]);g=sub(e,'geometry');sub(g,'cylinder',radius='.008',length='.04')
    tip=sub(robot,'link',name='sim_right_rod_tip');fixed(robot,'sim_right_rod_tip_joint','sim_right_rod','sim_right_rod_tip',[0,0,.04])
    # Gazebo camera uses X forward, Y left, Z up; calibration is optical X right,
    # Y down, Z forward. Keep the calibrated optical transform exactly.
    cam_to_opt=np.array([[0,0,1],[-1,0,0],[0,-1,0]],dtype=float)
    R=T[:3,:3]@cam_to_opt.T
    sensor=sub(robot,'link',name='sim_right_d405_link');inertial(sensor,.01)
    fixed(robot,'sim_right_d405_joint','right_gripper_flange','sim_right_d405_link',T[:3,3],Rotation.from_matrix(R).as_euler('xyz'))
    optical=sub(robot,'link',name='sim_right_d405_optical')
    fixed(robot,'sim_right_d405_optical_joint','sim_right_d405_link','sim_right_d405_optical',[0,0,0],Rotation.from_matrix(cam_to_opt).as_euler('xyz'))
    gz=sub(robot,'gazebo',reference='sim_right_d405_link');sub(gz,'preserveFixedJoint').text='true'
    sensor_el=sub(gz,'sensor',name='sim_right_d405',type='depth');sub(sensor_el,'always_on').text='true';sub(sensor_el,'update_rate').text='15';sub(sensor_el,'visualize').text='false'
    camera=sub(sensor_el,'camera');sub(camera,'horizontal_fov').text='1.16'
    img=sub(camera,'image');sub(img,'width').text='640';sub(img,'height').text='480';sub(img,'format').text='R8G8B8'
    clip=sub(camera,'clip');sub(clip,'near').text='.07';sub(clip,'far').text='2.0'
    plugin=sub(sensor_el,'plugin',name='sim_right_d405_ros',filename='libgazebo_ros_camera.so')
    ros=sub(plugin,'ros');sub(ros,'namespace').text='/sim/right_d405';sub(plugin,'camera_name').text='camera';sub(plugin,'frame_name').text='sim_right_d405_optical';sub(plugin,'min_depth').text='.07';sub(plugin,'max_depth').text='2.0'
    for obj,names in [('sim_hole_plate',['left_gripper_base','left_gripper_link1','left_gripper_link2']),('sim_right_rod',['right_gripper_base','right_gripper_link1','right_gripper_link2']),('sim_right_d405_link',['right_gripper_flange','right_gripper_base','right_link7'])]:
        for name in names:sub(semantic,'disable_collisions',link1=obj,link2=name,reason='Attached')
