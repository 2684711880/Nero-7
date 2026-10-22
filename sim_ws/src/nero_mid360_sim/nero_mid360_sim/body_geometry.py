"""Conservative per-collision oriented boxes, built from current URDF geometry."""
import math
import struct
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from ament_index_python.packages import get_package_share_directory

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


@lru_cache(maxsize=128)
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



class BodyModel:
    def __init__(self, description, padding=0.025):
        root = ET.fromstring(description)
        self.joints = []
        pending = root.findall('joint')
        known = {'base_link'}
        while pending:
            progressed = False
            for j in pending[:]:
                parent, child = j.find('parent').get('link'), j.find('child').get('link')
                if parent not in known:
                    continue
                axis = j.find('axis')
                self.joints.append((j.get('name'), parent, child, j.get('type'),
                    transform(j.find('origin')), np.fromstring(axis.get('xyz'), sep=' ') if axis is not None else np.zeros(3)))
                known.add(child); pending.remove(j); progressed = True
            if not progressed:
                raise ValueError('URDF must be one tree rooted at base_link')
        self.moving = [j[0] for j in self.joints if j[3] in ('revolute', 'continuous', 'prismatic')]
        self.boxes = []
        for link in root.findall('link'):
            for collision in link.findall('collision'):
                g = collision.find('geometry')
                cylinder, sphere = g.find('cylinder'), g.find('sphere')
                if cylinder is not None:
                    r, h = float(cylinder.get('radius')), float(cylinder.get('length'))/2
                    points = np.array([[-r,-r,-h], [r,r,h]])
                    T = transform(collision.find('origin'))
                elif sphere is not None:
                    r = float(sphere.get('radius')); points = np.array([[-r]*3, [r]*3])
                    T = transform(collision.find('origin'))
                else:
                    # Mesh bounds in collision frame, not the robot/world AABB.
                    raw = ET.fromstring(ET.tostring(collision))
                    origin = raw.find('origin')
                    if origin is not None: raw.remove(origin)
                    points = collision_points(raw)
                    T = transform(collision.find('origin'))
                lo, hi = points.min(axis=0), points.max(axis=0)
                if not np.isfinite(points).all(): raise ValueError('Invalid geometry')
                self.boxes.append((link.get('name'), T, (lo+hi)/2, (hi-lo)/2+padding))
        if 'mid360_link' not in known:
            raise ValueError('Missing mid360_link')

    def poses(self, q):
        result = {'base_link': np.eye(4)}
        for name, parent, child, kind, origin, axis in self.joints:
            T = origin.copy()
            if kind in ('revolute', 'continuous'):
                a = q[name]; axis = axis/np.linalg.norm(axis)
                K = np.array([[0,-axis[2],axis[1]], [axis[2],0,-axis[0]], [-axis[1],axis[0],0]])
                T[:3,:3] = T[:3,:3]@(np.eye(3)+math.sin(a)*K+(1-math.cos(a))*(K@K))
            elif kind == 'prismatic':
                T[:3,3] += T[:3,:3]@axis*q[name]
            result[child] = result[parent]@T
        return result

    def mask(self, points, q):
        poses = self.poses(q)
        sensor = poses['mid360_link']
        inside = np.zeros(len(points), dtype=bool)
        for link, origin, center, half in self.boxes:
            body = poses[link]@origin
            # Transform lidar points into the collision's local box coordinates.
            R = body[:3,:3].T@sensor[:3,:3]
            t = body[:3,:3].T@(sensor[:3,3]-body[:3,3])
            local = points@R.T+t-center
            inside |= (np.abs(local) <= half).all(axis=1)
        return inside
