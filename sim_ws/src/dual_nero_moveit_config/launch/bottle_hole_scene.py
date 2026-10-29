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
    # Bottle axis is across the gripper; local +Z points toward the bottom.
    body_radius, body_length, neck_radius, neck_length = .028, .15, .015, .04
    bottle = sub(robot, 'link', name='sim_bottle'); inertial(bottle, .1)
    fixed(robot, 'sim_bottle_joint', 'right_gripper_base', 'sim_bottle', [body_length/2 - .065, 0, .1058], [0, -math.pi/2, 0])
    for typ in ('visual', 'collision'):
        for name, radius, length, z in [('body', body_radius, body_length, body_length/2),
                                         ('neck', neck_radius, neck_length, -neck_length/2)]:
            e=sub(bottle,typ,name=name+'_'+typ);origin(e,[0,0,z]);g=sub(e,'geometry')
            sub(g,'cylinder',radius=str(radius),length=str(length))
            if typ == 'visual': sub(sub(e,'material',name='bottle_'+name),'color',rgba='.25 .65 .9 1')
    sub(robot,'link',name='sim_bottle_tip')
    fixed(robot,'sim_bottle_tip_joint','sim_bottle','sim_bottle_tip',[0,0,body_length])
    # Camera is on the bottle-neck (+X) side; use the source wrist mount.
    # Fixed middle camera: local Gazebo +X is forward, optical +Z is forward.
    cam_to_opt=np.array([[0,0,1],[-1,0,0],[0,-1,0]],dtype=float)
    sensor=sub(robot,'link',name='sim_middle_camera_link');inertial(sensor,.01)
    fixed(robot,'sim_middle_camera_joint','top_camera_stand_Link','sim_middle_camera_link',[.03,0,0],[0,.44,-.446])
    sub(robot,'link',name='sim_middle_camera_optical')
    fixed(robot,'sim_middle_optical_joint','sim_middle_camera_link','sim_middle_camera_optical',[0,0,0],Rotation.from_matrix(cam_to_opt).as_euler('xyz'))
    gz=sub(robot,'gazebo',reference='sim_middle_camera_link');sub(gz,'preserveFixedJoint').text='true'
    sensor_el=sub(gz,'sensor',name='sim_middle_camera',type='depth');sub(sensor_el,'always_on').text='true';sub(sensor_el,'update_rate').text='15';sub(sensor_el,'visualize').text='false'
    camera=sub(sensor_el,'camera');sub(camera,'horizontal_fov').text='1.16'
    img=sub(camera,'image');sub(img,'width').text='640';sub(img,'height').text='480';sub(img,'format').text='R8G8B8'
    clip=sub(camera,'clip');sub(clip,'near').text='.15';sub(clip,'far').text='3.0'
    plugin=sub(sensor_el,'plugin',name='sim_middle_camera_ros',filename='libgazebo_ros_camera.so')
    ros=sub(plugin,'ros');sub(ros,'namespace').text='/sim/middle';sub(plugin,'camera_name').text='camera';sub(plugin,'frame_name').text='sim_middle_camera_optical';sub(plugin,'min_depth').text='.15';sub(plugin,'max_depth').text='3.0'
    for obj,names in [('sim_hole_plate',['left_gripper_base','left_gripper_link1','left_gripper_link2']),('sim_bottle',['right_gripper_base','right_gripper_link1','right_gripper_link2']),('sim_middle_camera_link',['top_camera_stand_Link','base_link','camera_link'])]:
        for name in names:sub(semantic,'disable_collisions',link1=obj,link2=name,reason='Attached')
