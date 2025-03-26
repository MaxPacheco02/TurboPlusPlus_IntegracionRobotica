#!/usr/bin/env python3

import cv2
import numpy as np


class ARU:

    def __init__(self):
        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }

        # PARA PUNTOS EN EL ARUCO
        self.PBUV = np.zeros((2,2),np.float32)

        # PARA VECTOR DE ROTACION DE ARUCO
        self.aru_ROT = np.zeros((3,1),np.float32)

        # TAMANO DE ARUCO EN MM
        self.markerLength = 150

        # DICCIONARIO DE ARUCO
        self.aruco_dict = self.ARUCO_DICT["DICT_4X4_50"]

        # CORNERS EN EL ARUCO, CON RESPECTO AL CENTRO
        self.objp = np.array([[-self.markerLength/2, self.markerLength/2,  0],
                            [  self.markerLength/2, self.markerLength/2,  0],
                            [  self.markerLength/2, -self.markerLength/2, 0],
                            [ -self.markerLength/2, -self.markerLength/2, 0]],np.float32)

        # FLAG PARA DETERMINAR OBJETO O PERSONA
        self.flag = 0.0


    def pose_estimation(self, frame, matrix_coefficients, distortion_coefficients):
        markerLength = 81
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict)
        parameters = cv2.aruco.DetectorParameters()
        
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        
        corners, ids, rejectedCandidates = detector.detectMarkers(gray)
        
        cor = 0
        if len(corners) > 0:
            for i in range(0, len(ids)):
                _, rvec, tvec = cv2.solvePnP(self.objp, corners[i], matrix_coefficients, distortion_coefficients)
                cor = corners[i]
                
                cv2.aruco.drawDetectedMarkers(frame, corners, ids) 
                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, markerLength*0.4,4)  

                self.PBUV[0] = cor[0][0]
                self.PBUV[1] = cor[0][1]
                self.aru_ROT = rvec

        return frame, self.PBUV, self.aru_ROT
    
            
