"""Offline pointing geometry; never opens cameras or CAN."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).parents[1]))
import handeye_charuco as h
import point_charuco as point
from import_handeye import read_launch


class PointTests(unittest.TestCase):
    def test_standoff_and_tcp_are_applied_in_their_own_frames(self):
        camera = h.transform(translation=[.03,-.02,.07])
        board = h.transform(np.diag([1.,-1.,-1.]),[.1,.2,.6])
        result = point.pointing_target(np.eye(4),camera,board,[0.,.2,0.],.1,[0,0,.2])
        np.testing.assert_allclose(result['corner_base_m'],[.13,-.02,.67],atol=1e-12)
        np.testing.assert_allclose(result['target_tcp_base_m'],[.13,-.02,.57],atol=1e-12)
        np.testing.assert_allclose(result['target_flange_base_m'],[.13,-.02,.37],atol=1e-12)

    def test_offset_rotates_with_flange(self):
        gripper=h.transform(h.rpy_matrix([0.,0.,np.pi/2]))
        board=h.transform(np.diag([1.,-1.,-1.]),[0.,0.,.6])
        result=point.pointing_target(gripper,np.eye(4),board,[0,0,0],.1,[.2,0,0])
        np.testing.assert_allclose(np.array(result['target_tcp_base_m'])-result['target_flange_base_m'],[0,.2,0],atol=1e-12)

    def test_user_portrait_board_top_left(self):
        b=h.Board(dict(squares_x=7,squares_y=5,square_m=.04,marker_m=.03,dictionary='DICT_5X5_50'))
        np.testing.assert_allclose(point.select_corner(b,'board'),[0,0,0])
        np.testing.assert_allclose(point.select_corner(b,'marker',0),b.board.objPoints[0][0])
        with self.assertRaises(ValueError):point.select_corner(b,'marker')

    def test_portrait_projection_selects_left_not_right_top(self):
        import cv2
        b=h.Board(dict(squares_x=7,squares_y=5,square_m=.04,marker_m=.03,dictionary='DICT_5X5_50'))
        r=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
        k=np.array([[650.,0.,640.],[0.,650.,360.],[0.,0.,1.]])
        corners=np.array([[0.,0.,0.],[0.,.2,0.],[.28,0.,0.],[.28,.2,0.]])
        uv=cv2.projectPoints(corners,cv2.Rodrigues(r)[0],np.array([-.1,-.14,.6]),k,None)[0].reshape(-1,2)
        chosen=point.select_corner(b,'board')
        index=np.flatnonzero(np.all(corners==chosen,axis=1))[0]
        self.assertEqual(index,0)
        self.assertLess(uv[index,0],uv[1,0])
        self.assertLess(uv[index,1],uv[2,1])

    def test_imported_result_is_camera_to_flange(self):
        x=read_launch(h.ROOT/'archive/handeye/earlier_sessions/Nero1.launch.py')
        np.testing.assert_allclose(x[:3,3],[.0673125,-.00907703,.0541585])
        np.testing.assert_allclose(x[:3,:3].T@x[:3,:3],np.eye(3),atol=1e-12)


if __name__ == '__main__':unittest.main()
