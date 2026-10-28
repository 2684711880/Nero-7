#!/usr/bin/env python3
"""Conservative arm-geometry radius bound over joint angles, 0..0.8 m lift and lidar FOV.

Triangle inequality bounds all arm collision vertices, including gripper openings
and wrist cameras. No hardware reach, ray visibility, held objects or accuracy claim.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from ament_index_python.packages import get_package_share_directory
from mobile_dual_nero.model import compose, RELAXED_ARM, LIFT_MAX
from mobile_dual_nero.mid360 import mount_pose


def rpy_matrix(rpy):
    r,p,y=rpy;cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def transform(origin):
    T=np.eye(4)
    if origin is not None:
        T[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        T[:3,:3]=rpy_matrix(np.fromstring(origin.get('rpy','0 0 0'),sep=' '))
    return T


def mesh_vertices(uri):
    package,relative=uri.removeprefix('package://').split('/',1)
    path=Path(get_package_share_directory(package))/relative
    if path.suffix.lower()=='.stl':
        data=path.read_bytes();n=struct.unpack_from('<I',data,80)[0]
        if len(data)==84+50*n:
            dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])
            return np.frombuffer(data,dtype=dtype,offset=84,count=n)['vertices'].reshape(-1,3).astype(float)
        rows=[line.split()[1:] for line in data.decode().splitlines() if line.strip().startswith('vertex ')]
        return np.array(rows,dtype=float)
    if path.suffix.lower()=='.dae':
        root=ET.parse(path).getroot();ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
        geometries={}
        for geometry in root.findall('c:library_geometries/c:geometry',ns):
            mesh=geometry.find('c:mesh',ns)
            ref=mesh.find("c:vertices/c:input[@semantic='POSITION']",ns).get('source')[1:]
            source=mesh.find(f"c:source[@id='{ref}']",ns)
            geometries[geometry.get('id')]=np.fromstring(source.findtext('c:float_array',namespaces=ns),sep=' ').reshape(-1,3)
        points=[]
        def walk(node,parent):
            T=parent.copy()
            for child in node:
                if child.tag.endswith('}matrix'):
                    T=T@np.fromstring(child.text,sep=' ').reshape(4,4).T
                elif child.tag.endswith(('}translate','}scale','}rotate')):
                    raise ValueError('Unsupported Collada transform; extend parser before using this bound')
            for item in node.findall('c:instance_geometry',ns):
                v=geometries[item.get('url')[1:]];points.append(v@T[:3,:3].T+T[:3,3])
            for child in node.findall('c:node',ns):walk(child,T)
        scene=root.find('c:library_visual_scenes/c:visual_scene',ns)
        for node in scene.findall('c:node',ns):walk(node,np.eye(4))
        unit=root.find('c:asset/c:unit',ns)
        return np.concatenate(points)*float(unit.get('meter','1'))
    raise ValueError(f'Unsupported collision mesh {path}')


def collision_points(collision):
    geometry=collision.find('geometry');mesh=geometry.find('mesh')
    if mesh is not None:
        points=mesh_vertices(mesh.get('filename'))*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
    elif geometry.find('box') is not None:
        size=np.fromstring(geometry.find('box').get('size'),sep=' ')/2
        points=np.array([[x,y,z] for x in [-size[0],size[0]] for y in [-size[1],size[1]] for z in [-size[2],size[2]]])
    else:raise ValueError('Unsupported arm collision primitive')
    T=transform(collision.find('origin'))
    return points@T[:3,:3].T+T[:3,3]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=Path('sim_ws/validation/lidar_arm_blind/report.json'))
    a=p.parse_args();root=ET.fromstring(compose()[0]);joints=root.findall('joint')
    parent={j.find('child').get('link'):j for j in joints}
    def poses(relaxed=False, positions=None):
        result={'base_link':np.eye(4)};todo=joints[:]
        while todo:
            for joint in todo[:]:
                name=joint.get('name');source=joint.find('parent').get('link');target=joint.find('child').get('link')
                if source not in result:continue
                T=transform(joint.find('origin'));angle=0.
                if relaxed and any(name==f'{side}_joint{i}' for side in ['left','right'] for i in range(1,8)):
                    angle=RELAXED_ARM[int(name[-1])-1]
                if positions is not None:
                    angle=positions.get(name,0.)
                if joint.get('type')=='prismatic':
                    value=angle
                    if relaxed and positions is None:
                        value=float(root.findtext(f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param[@name='initial_value']",'0'))
                    axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                    T[:3,3]+=T[:3,:3]@axis*value
                if joint.get('type')=='revolute' and angle:
                    axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                    K=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
                    T[:3,:3]=T[:3,:3]@(np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K))
                result[target]=result[source]@T;todo.remove(joint)
        return result
    zero=poses();relaxed=poses(True);link_bounds=[]
    for link in root.findall('link'):
        name=link.get('name');side='left' if name.startswith('left_') else 'right' if name.startswith('right_') else None
        if side is None:continue
        for collision in link.findall('collision'):
            points=collision_points(collision)
            if not len(points) or not np.isfinite(points).all():raise ValueError(name)
            # left/right_link1 is the fixed shoulder-2 pivot; joint1 rotates about it.
            anchor=side+'_link1';cursor=name;path_length=0.;descendant=True
            while cursor!=anchor:
                if cursor not in parent:descendant=False;break
                joint=parent[cursor];path_length+=np.linalg.norm(transform(joint.find('origin'))[:3,3])
                if joint.get('type')=='prismatic':
                    lim=joint.find('limit');path_length+=max(abs(float(lim.get('lower'))),abs(float(lim.get('upper'))))
                cursor=joint.find('parent').get('link')
            if not descendant:
                # Proximal arm base is rigidly fixed to the robot.
                if name!=side+'_arm_base':raise ValueError(f'Unexpected arm collision branch {name}')
            link_bounds.append(dict(name=name,local_points=points,anchor=anchor,anchor_position=zero[anchor][:3,3],
                radius=float(path_length+np.linalg.norm(points,axis=1).max()) if descendant else None,
                relaxed_points=points@relaxed[name][:3,:3].T+relaxed[name][:3,3],
                fixed_points=points@zero[name][:3,:3].T+zero[name][:3,3]))
    results={}
    for preset in ['chassis_up','chest_down']:
        xyz,rpy=mount_pose(preset);sensor=np.array(xyz);rows=[]
        for item in link_bounds:
            bounds=[]
            for height in [0., LIFT_MAX]:
                shift=np.array([0.,0.,height]); sensor_at_height=sensor+(shift if preset=='chest_down' else 0)
                bounds.append(float(np.linalg.norm(sensor_at_height-item['anchor_position']-shift)+item['radius'])
                    if item['radius'] is not None else float(np.linalg.norm(item['fixed_points']+shift-sensor_at_height,axis=1).max()))
            bound=max(bounds)
            rows.append({'link':item['name'],'all_angles_upper_bound_m':bound,'relaxed_max_surface_distance_m':float(np.linalg.norm(item['relaxed_points']-sensor,axis=1).max())})
        bound=max(r['all_angles_upper_bound_m'] for r in rows)
        recommended=math.ceil((bound+.05)/.05)*.05
        results[preset]=dict(xyz=xyz,rpy=rpy,lift_range_m=[0.,LIFT_MAX],arm_collision_links=len(rows),all_angles_upper_bound_m=bound,
            recommended_blind_with_5cm_margin_m=round(recommended,2),
            relaxed_max_surface_distance_m=max(r['relaxed_max_surface_distance_m'] for r in rows),per_link=rows)
    rng=np.random.default_rng(258);sampled={k:0. for k in results}
    for _ in range(100):
        positions={}
        for joint in joints:
            limit=joint.find('limit')
            if joint.get('name').startswith(('left_','right_')) and limit is not None and limit.get('lower') is not None:
                positions[joint.get('name')]=rng.uniform(float(limit.get('lower')),float(limit.get('upper')))
        positions['upper_body_lift_joint']=rng.uniform(0,LIFT_MAX)
        state=poses(positions=positions)
        for item in link_bounds:
            points=item['local_points'][::max(1,len(item['local_points'])//100)]
            T=state[item['name']];points=points@T[:3,:3].T+T[:3,3]
            for preset,row in results.items():
                sensor=np.array(row['xyz'])+np.array([0.,0.,positions['upper_body_lift_joint'] if preset=='chest_down' else 0.])
                distance=float(np.linalg.norm(points-sensor,axis=1).max())
                sampled[preset]=max(sampled[preset],distance)
                assert distance<=row['all_angles_upper_bound_m']+1e-6,(preset,item['name'],distance)
    for preset,row in results.items():
        row['random_configuration_sanity_check_max_m']=sampled[preset]
        h=row['xyz'][2];angle=7. if preset=='chassis_up' else 52.
        row['ideal_nearest_ground_horizontal_m']=h/math.tan(math.radians(angle))
        blind=row['recommended_blind_with_5cm_margin_m']
        row['ground_horizontal_min_after_recommended_blind_m']=max(row['ideal_nearest_ground_horizontal_m'],math.sqrt(max(blind*blind-h*h,0)))
    report=dict(random_configuration_sanity_checks=100, method='Triangle inequality from shoulder pivot over both 0..0.8 m lift endpoints; all arm collision vertices, grippers and wrist cameras; no FOV restriction or collision-free-state assumption needed',
        limitations=['Default robot mounting geometry only','No held object, cable or unmodelled parts','Bounds direct arm returns only; cannot remove occlusion or motion-induced vibration','This is an offline geometry bound, not an executed motion/raycast validation'],results=results)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    (a.output.parent/'model.urdf').write_text(ET.tostring(root,encoding='unicode'))
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:{f:v for f,v in row.items() if f!='per_link'} for k,row in results.items()},indent=2))


if __name__=='__main__':main()
