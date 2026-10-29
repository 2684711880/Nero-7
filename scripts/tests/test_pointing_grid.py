import sys
import unittest
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]))
from pointing_grid import GridBoard


class GridTest(unittest.TestCase):
    def test_board_geometry_and_id_zero_corner(self):
        board=GridBoard()
        self.assertEqual(board.ids.tolist(),list(range(12)))
        np.testing.assert_allclose(board.corner,[0,.129,0],atol=1e-7)
        np.testing.assert_allclose([board.width,board.height],[.096,.129])

    def test_drawn_board_recovers_pose(self):
        board=GridBoard()
        image=board.board.draw((960,1290),marginSize=30,borderBits=1)
        image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        k=np.array([[900.,0.,480.],[0.,900.,645.],[0.,0.,1.]])
        result,_=board.detect(image,k,np.zeros(5))
        self.assertIsNotNone(result)
        self.assertEqual(len(result['marker_ids']),12)
        self.assertLess(result['reprojection_rms_px'],1.)
        self.assertGreater(result['T_camera_board'][2][3],0)
