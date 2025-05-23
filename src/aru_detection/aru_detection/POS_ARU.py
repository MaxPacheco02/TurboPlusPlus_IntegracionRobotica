#!/usr/bin/env python3

import cv2
import numpy as np
from collections import deque, defaultdict

class ARU:

    def __init__(self):
        
        # PARA VECTOR DE ROTACION DE ARUCO
        self.aru_ROT = np.zeros((3,1))

        # PARA VECTOR DE TRASLACION DE ARUCO
        self.aru_TRAN = np.zeros((3,1))

        # TAMANO DE ARUCO EN MM
        self.markerLength = 50

        # DICCIONARIO DE ARUCO
        self.aruco_dict = cv2.aruco.DICT_4X4_50

        # CORNERS EN EL ARUCO, CON RESPECTO AL CENTRO
        self.objp = np.array([[-self.markerLength/2, self.markerLength/2,  0],
                            [  self.markerLength/2, self.markerLength/2,  0],
                            [  self.markerLength/2, -self.markerLength/2, 0],
                            [ -self.markerLength/2, -self.markerLength/2, 0]],np.float32)

        # FLAG PARA DETERMINAR OBJETO O PERSONA
        self.flag = 0.0

        self.aruList = []

        self.aruSeen = defaultdict(lambda: deque(maxlen=10))


    def detectorSetup(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)


    def pose_estimation(self, frame, matrix_coefficients, distortion_coefficients):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, _ = self.detector.detectMarkers(gray)

        self.aruList.clear()
        
        rvec = 0
        tvec = 0
        if(len(corners) > 0):
            for i in range(0, len(ids)):
                marker_id = ids[i][0]

                self.aruSeen[marker_id].append(True)

                if(len(self.aruSeen[marker_id]) == 10 and all(self.aruSeen[marker_id])):
            
                    _, rvec, tvec = cv2.solvePnP(self.objp, corners[i], matrix_coefficients, distortion_coefficients)
                    
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids) 
                    cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 20,2)  

                    # [ROTATION | TRANSLATION | ID]
                    detectedAru = [0,0,0]

                    detectedAru[0] = rvec
                    detectedAru[1] = tvec
                    detectedAru[2] = ids[i][0]

                    self.aru_ROT = rvec
                    self.aru_TRAN = tvec

                    self.aruList.append(detectedAru)
        else: 
            # SI LA CAMARA PIERDE EL ARUCO QUEDARSE CON LA ULTIMA POSICION VISTA
            self.aruList = [[[0,0,0],[0,0,0],-1]]
            self.aruSeen.clear()

        return frame, self.aruList
    
            
