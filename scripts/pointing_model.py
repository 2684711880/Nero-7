"""Dual calibrated NERO chains and torso, reusing official MoveIt/controllers."""
import copy
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]


def compose_single(offset, initial):
    official = Path(get_package_share_directory('agx_arm_moveit'))/'launch'
    sys.path.insert(0,str(official))
    spec=importlib.util.spec_from_file_location('pointing_official_builder',official/'_moveit_config_builder.py')
    builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
    ctx=LaunchContext()
    ctx.launch_configurations.update(arm_type='nero',effector_type='none',revo2_type='left',tcp_offset='[0,0,0,0,0,0]')
    config=builder.build_moveit_config(ctx).to_dict()
    robot=ET.parse(ROOT/'config/gravity/nero_standard_gripper_no_camera.urdf').getroot()
    meshes=ROOT/'third_party/open_nero/open_nero_description/meshes/nero/meshes'
    for mesh in robot.findall('.//mesh'):
        original=Path(mesh.get('filename'))
        candidate=meshes/('dae/'+original.name if original.suffix=='.dae' else original.name)
        if not candidate.is_file():raise FileNotFoundError(candidate)
        mesh.set('filename',candidate.as_uri())
    # Preserve the exact FK used during calibration. Add only the configured tip.
    ET.SubElement(robot,'link',name='point_tcp')
    joint=ET.SubElement(robot,'joint',name='point_tcp_joint',type='fixed')
    ET.SubElement(joint,'parent',link='gripper_flange');ET.SubElement(joint,'child',link='point_tcp')
    ET.SubElement(joint,'origin',xyz=' '.join(map(str,offset)),rpy='0 0 0')
    control=ET.SubElement(robot,'ros2_control',name='PointingTrajectoryBuffer',type='system')
    hw=ET.SubElement(control,'hardware');ET.SubElement(hw,'plugin').text='mock_components/GenericSystem'
    for i,q in enumerate(initial,1):
        j=ET.SubElement(control,'joint',name=f'joint{i}')
        ET.SubElement(j,'command_interface',name='position')
        state=ET.SubElement(j,'state_interface',name='position')
        ET.SubElement(state,'param',name='initial_value').text=str(q)
        ET.SubElement(j,'state_interface',name='velocity')
    srdf=ET.Element('robot',name=robot.get('name'))
    group=ET.SubElement(srdf,'group',name='arm')
    ET.SubElement(group,'chain',base_link='base_link',tip_link='point_tcp')
    # Adjacent links and links on the same rigid assembly need not collide.
    fixed={j.find('child').get('link'):j.find('parent').get('link') for j in robot.findall('joint') if j.get('type')=='fixed'}
    def root(name):
        while name in fixed:name=fixed[name]
        return name
    adjacent=set()
    for j in robot.findall('joint'):
        adjacent.add(frozenset((root(j.find('parent').get('link')),root(j.find('child').get('link')))))
    links=[l.get('name') for l in robot.findall('link')]
    for i,a in enumerate(links):
        for b in links[i+1:]:
            if root(a)==root(b) or frozenset((root(a),root(b))) in adjacent:
                ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='Adjacent')
    config.update(robot_description=ET.tostring(robot,encoding='unicode'),
                  robot_description_semantic=ET.tostring(srdf,encoding='unicode'),
                  robot_description_kinematics={'arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin',
                    'kinematics_solver_timeout':.2,'kinematics_solver_search_resolution':.005}},
                  allow_trajectory_execution=True,
                  publish_robot_description=True,publish_robot_description_semantic=True,
                  publish_planning_scene=True,publish_geometry_updates=True,
                  publish_state_updates=True,publish_transforms_updates=True)
    config['robot_description_planning']['joint_limits']={f'joint{i}':dict(has_velocity_limits=True,max_velocity=1.,
        has_acceleration_limits=True,max_acceleration=1.) for i in range(1,8)}
    config.pop('sensors',None)
    config.pop('point_cloud_sensor',None)
    controllers={'controller_manager':{'ros__parameters':{'update_rate':200,
        'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'},
        'arm_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'}}},
        'arm_controller':{'ros__parameters':{'joints':[f'joint{i}' for i in range(1,8)],
            'command_interfaces':['position'],'state_interfaces':['position','velocity']}}}
    return config, controllers


def mount_matrix(side):
    import numpy as np
    from handeye_charuco import transform,rpy_matrix
    mounts=ET.parse(ROOT/'third_party/open_nero/open_nero_description/urdf/dual_nero.xacro').getroot()
    o=mounts.find(f"joint[@name='{side}_arm_joint']/origin")
    return transform(rpy_matrix(np.fromstring(o.get('rpy'),sep=' ')),np.fromstring(o.get('xyz'),sep=' '))


def compose(offset,initial,right_initial=None):
    config,_=compose_single(offset,initial)
    single=ET.fromstring(config['robot_description'])
    robot=ET.Element('robot',name='dual_nero_pointing')
    assets=ROOT/'third_party/open_nero/open_nero_description'
    fixture=ET.parse(assets/'urdf/open_nero_description.urdf').getroot()
    body=copy.deepcopy(fixture.find("link[@name='base_link']"))
    for mesh in body.findall('.//mesh'):
        path=assets/mesh.get('filename').removeprefix('package://open_nero_description/')
        if not path.is_file():raise FileNotFoundError(path)
        mesh.set('filename',path.as_uri())
    robot.append(body)
    srdf=ET.Element('robot',name='dual_nero_pointing')
    one_srdf=ET.fromstring(config['robot_description_semantic'])
    mounts=ET.parse(assets/'urdf/dual_nero.xacro').getroot()
    for side in ('left','right'):
        prefix=side+'_'
        for original in single:
            if original.tag=='link' and original.get('name')=='world':continue
            if original.tag=='joint' and original.find("parent[@link='world']") is not None:continue
            if original.tag=='ros2_control' and side=='right':continue
            child=copy.deepcopy(original)
            for e in child.iter():
                if e.tag in ('link','joint','ros2_control') and 'name' in e.attrib:e.set('name',prefix+e.get('name'))
                if e.tag in ('parent','child') and 'link' in e.attrib:e.set('link',prefix+e.get('link'))
            robot.append(child)
        mount=copy.deepcopy(mounts.find(f"joint[@name='{side}_arm_joint']"))
        mount.find('child').set('link',prefix+'base_link');robot.append(mount)
        for original in one_srdf:
            child=copy.deepcopy(original)
            if child.tag=='group':
                child.set('name',prefix+'arm')
                child.find('chain').set('base_link',prefix+'base_link')
                child.find('chain').set('tip_link',prefix+'point_tcp')
            elif child.tag=='disable_collisions':
                a,b=child.get('link1'),child.get('link2')
                if 'world' in (a,b):continue
                child.set('link1',prefix+a);child.set('link2',prefix+b)
            srdf.append(child)
        # The torso touches the mount and first link; other arm/body pairs stay checked.
        for link in ('base_link','link1'):
            ET.SubElement(srdf,'disable_collisions',link1='base_link',link2=prefix+link,reason='Adjacent')
    kinematics=config['robot_description_kinematics']['arm']
    config.update(robot_description=ET.tostring(robot,encoding='unicode'),
                  robot_description_semantic=ET.tostring(srdf,encoding='unicode'),
                  robot_description_kinematics={s+'_arm':copy.deepcopy(kinematics) for s in ('left','right')})
    limits=config['robot_description_planning']['joint_limits']
    config['robot_description_planning']['joint_limits']={s+'_'+j:copy.deepcopy(v) for s in ('left','right') for j,v in limits.items()}
    for pipeline in config.get('planning_pipelines',[]):
        if pipeline in config and 'arm' in config[pipeline]:
            arm=config[pipeline].pop('arm')
            config[pipeline].update(left_arm=copy.deepcopy(arm),right_arm=copy.deepcopy(arm))
    names=[f'left_joint{i}' for i in range(1,8)]
    config['moveit_simple_controller_manager']={'controller_names':['left_arm_controller'],
        'left_arm_controller':dict(type='FollowJointTrajectory',action_ns='follow_joint_trajectory',default=True,joints=names)}
    controllers={'controller_manager':{'ros__parameters':{'update_rate':200,
        'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'},
        'left_arm_controller':{'type':'joint_trajectory_controller/JointTrajectoryController'}}},
        'left_arm_controller':{'ros__parameters':{'joints':names,'command_interfaces':['position'],
                                                   'state_interfaces':['position','velocity']}}}
    return config,controllers
