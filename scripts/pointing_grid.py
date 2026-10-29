"""3x4 ArUco GridBoard geometry used by the right wrist pointing preview."""
import cv2
import numpy as np
from handeye_charuco import transform


class GridBoard:
    def __init__(self):
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.board=cv2.aruco.GridBoard_create(3,4,.030,.003,self.dictionary,firstMarker=0)
        self.width=.096
        self.height=.129
        self.ids=np.asarray(self.board.ids).flatten()
        self.object_points=self.board.objPoints
        # The first corner of ID 0 is the canonical printed top-left.
        self.corner=np.asarray(self.object_points[int(np.flatnonzero(self.ids==0)[0])][0],dtype=float)

    def detect(self,image,k,dist):
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        params=cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        corners,ids,_=cv2.aruco.detectMarkers(gray,self.dictionary,parameters=params)
        preview=image.copy()
        if ids is None:return None,preview
        cv2.aruco.drawDetectedMarkers(preview,corners,ids)
        obj=[]; pixels=[]; used=[]
        for c,marker in zip(corners,ids.flatten()):
            match=np.flatnonzero(self.ids==marker)
            if len(match)==1:
                obj.extend(np.asarray(self.object_points[int(match[0])]).reshape(4,3))
                pixels.extend(np.asarray(c).reshape(4,2));used.append(int(marker))
        if len(used)<2:return None,preview
        obj=np.asarray(obj,dtype=np.float32);pixels=np.asarray(pixels,dtype=np.float32)
        ok,r,t=cv2.solvePnP(obj,pixels,k,dist)
        if not ok or t[2,0]<=0:return None,preview
        projected=cv2.projectPoints(obj,r,t,k,dist)[0].reshape(-1,2)
        error=float(np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1))))
        return dict(marker_ids=used,T_camera_board=transform(cv2.Rodrigues(r)[0],t).tolist(),
                    reprojection_rms_px=error),preview
