#!/usr/bin/env python3

import cv2
import glob
import numpy as np
import json
import os

from ament_index_python.packages import get_package_share_directory

# TOMAR FOTOS Y CALIBRAR CAMARA

class CAM:

    def __init__(self):
        # MATRICES Y VECTORES OBTENIDOS DE CALIBRACION DE CAMARA
        self.CamMat = 0
        self.CamTra = 0
        self.CamRot = 0
        self.CamDis = 0


    # TOMAR FOTOS DE CHESSBOARD PARA CALIBRACION (*AHORITA NO LAS GUARDA)
    def imageCapture(self):
        IMG = cv2.VideoCapture(0)
        num = 'A'

        while IMG.isOpened():
            ret, frame = IMG.read()
            k = cv2.waitKey(1)
            if k == ord('p'): 
                S = 'chess' + num + '.png'
                print("images saved!")
                # num += 1
            elif k == ord('q'):
                break
            cv2.imshow('Image',frame)
        IMG.release()
        cv2.destroyAllWindows()


    def cameraCalibrate(self):
        # NUMERO DE ESQUINAS EN CHESSBOARD DE CALIBRACION
        chessSize = (5,5)
        # TAMAÑO DE CADA CUADRO EN MM
        chessSq = 31
        # TERMINATION CRITERIA
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # PUNTOS PARA PLANOS DE CALIBRACION
        objp = np.zeros((chessSize[0]*chessSize[0],3), np.float32)
        objp[:,:2] = np.mgrid[0:chessSize[0],0:chessSize[1]].T.reshape(-1,2)

        # ------------------------------------------------------------------------------
        #ACOMODAR PUNTOS PARA MANTENER REFERENCIAL SIN TENER QUE MODIFICAR SUS SIGNOS
        for i in range(chessSize[0]-1,len(objp),chessSize[0]):
            TEMP = objp[i-(chessSize[0]-1):i+1,:] 
            objp[i-(chessSize[0]-1):i+1,:] = TEMP[::-1]
            
            #  | y               #
            #  |                 #
            #  |                 #
            #  |                 #
            #  |                 #
            #  . z ___________x  #
        # ------------------------------------------------------------------------------

        # ASIGNAR MEDIDAS REALES A OBJP (MM)
        objp = objp * chessSq
        # ARREGLOS DE PUNTOS EN IMAGEN Y CORRESPONDENCIAS EN EL MUNDO
        objpoints = [] # 3d 
        imgpoints = [] # 2d

        # OBTENER PATH DE DIRECTORIO DE FOTOS (EN DIRECTORIO SHARE DEL PAQUETE)
        share_dir = get_package_share_directory('service')
        chessboard_path = os.path.join(share_dir, 'resources')

        # IMPORTAR IMAGENES PARA CALIBRACION
        images = glob.glob(os.path.join(chessboard_path, '*.png'))

        for fname in images:
            img = cv2.imread(fname)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, chessSize, None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                
                # Draw and display the corners
                cv2.drawChessboardCorners(img, chessSize, corners2, ret)
                # cv2.imshow('Imagen', img)
                # cv2.waitKey(0)

        cv2.destroyAllWindows()

        #CALIBRACION DE CAMARA
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, None)
        self.CamMat = mtx
        self.CamDis = dist
        self.CamRot = rvecs
        self.CamTra = tvecs
    