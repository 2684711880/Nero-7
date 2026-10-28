"""Offline checks for point-frame conversion and capture file contract."""
from pathlib import Path
from types import SimpleNamespace
import json
import sys
import tempfile
import unittest
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).parents[1]))
from icp_handeye import depth_to_color,select_points,save_sample

class ICPInputTests(unittest.TestCase):
    def test_depth_to_color_matches_realsense_sdk_column_order(self):
        r=Rotation.from_euler('xyz',[.1,-.2,.3]).as_matrix()
        ext=rs.extrinsics();ext.rotation=r.flatten(order='F').tolist();ext.translation=[.01,-.02,.03]
        t=depth_to_color(ext);p=np.array([.03,.02,.3])
        np.testing.assert_allclose((t@np.r_[p,1])[:3],rs.rs2_transform_point_to_point(ext,p.tolist()),atol=1e-7)

    def test_selection_rejects_bad_depth_and_transforms_color_frame(self):
        points=np.zeros((25,25,3),np.float32);points[:]=[.02,.03,.3];points[0,:]=0
        t=np.eye(4);t[:3,3]=[.01,-.02,.03]
        result=select_points(points,(0,0,25,25),t,.07,.6)
        self.assertEqual(len(result),600)
        np.testing.assert_allclose(result[0],[.03,.01,.33],atol=1e-7)
        with self.assertRaises(ValueError):select_points(points,(0,0,5,5),t,.07,.6)

    def test_written_pose_rows_and_points_retain_correspondence(self):
        with tempfile.TemporaryDirectory() as temp:
            d=Path(temp);points=np.tile(np.array([[.1,.2,.3]],dtype='<f4'),(300,1));q=np.zeros(7)
            p=np.eye(4);p[:3,:3]=Rotation.from_euler('xyz',[.1,.2,-.3]).as_matrix();p[:3,3]=[.2,-.3,.4]
            image=np.zeros((4,4),np.uint16);color=np.zeros((4,4,3),np.uint8)
            save_sample(d,1,points,p,q,{},[0,0,4,4],image,color)
            row=np.loadtxt(d/'RobotPoses.dat',delimiter=',')
            np.testing.assert_allclose(Rotation.from_euler('xyz',row[:3]).as_matrix(),p[:3,:3],atol=1e-10)
            np.testing.assert_allclose(row[3:],[200,-300,400],atol=1e-10)
            payload=(d/'view1d.pcd').read_bytes().split(b'DATA binary\n',1)[1]
            np.testing.assert_array_equal(np.frombuffer(payload,dtype='<f4').reshape(-1,3),points)
            self.assertEqual(json.loads((d/'view1.json').read_text())['point_frame'],'handeye_color_optical')

if __name__=='__main__':unittest.main()
