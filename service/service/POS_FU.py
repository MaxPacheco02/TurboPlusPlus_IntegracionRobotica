#!/usr/bin/env python3

import cv2
import numpy as np

from .POS_CAL import CAM

class POS:

    def __init__(self, CAM, ImgNum):

        # INDICE DE IMAGEN PARA REFFERENCIAL DE MUNDO
        self.ImgNum = ImgNum

        # MATRICES Y VECTORES
        self.R, _ = cv2.Rodrigues(CAM.CamRot[ImgNum])
        self.T = CAM.CamTra[ImgNum]
        self.K = CAM.CamMat

        # CONSTRUIR MATRIZ DE VALORES EXTRINSECOS P 4x4 (MATRIZ DE TRANSFORMACION HOMOGENEA)
        self.H = np.concatenate((np.concatenate((self.R , self.T), axis=1),[[0,0,0,1]]), axis=0)


        # PARA GRAFICAR [ARREGLADO] (YA NO SE OCUPA CAMBIAR SIGNOS)
        # ---------------------------
        # POSICION ORIGEN DE MUNDO
        self.Pw = np.array([[0, 0, 0, 0]]).T
        # FRAME DE MUNDO
        self.Rw = np.array([[62, 0, 0], [0, 62, 0], [0, 0, 62]])
        # POSICION DE REFERENCIAL DE CAMARA
        self.Pc = 0
        # FRAME DE CAMARA
        self.Rc = 0
        # TRASLACION Y ROTACION
        self.Tg = np.copy(self.T)
        self.Rg = self.R
        # ---------------------------


    # CALCULAR POSICION DE CAMARA Y MUNDO
    def worldCal(self):
        Tg = np.concatenate((self.Tg,[[1]]),axis=0)
        # PERFORM ROTATION
        self.Rc = self.Rg @ self.Rw
        # TRASLACION 
        self.Pc = self.Pw + Tg
        return self.Rg, self.Pc #Original self.Rc, self.Pc


    # CALCULAR POSICION DE OBJETO 
    # PUV es el vector 3x1 del punto en la imagen [u,v,1].T
    # Zc es la distancia estimada del referencial de la camara al objeto
    def objCal(self,PUV,Zc):
        return self.planeToWorld(self.R,self.T,self.K,PUV,Zc)
    
    
    # CALCULAR POSICION DE OBJETO EN PLANO DE IMAGEN
    # W es un vector en el mundo [Xw,Yw,Zw,1].T
    def planCal(self,W):
        return self.worldToPlane(W,self.H,self.K)
    

    def worldToPlane(self,W,H,K):
        # CONSTRUIR MATRIZ INTRINSECA
        M = np.concatenate((K,np.array([[0,0,0]]).T), axis=1)
        # DE REFERENCIAL DE MUNDO A REFERENCIAL DE CAMARA
        C = H @ W
        print("\nC_p = \n",C)
        # DE REFERENCIAL DE CAMARA A PLANO DE IMAGEN
        V = M @ C
        # PUNTO EN PIXELES EN PLANO DE IMAGEN (u,v) -> (X,Y)
        u = V[0]/V[2]  # Xc / Zc
        v = V[1]/V[2]  # Yc / Zc
        return [round(np.ravel(u)[0],0),round(np.ravel(v)[0],0)]


    def planeToWorld(self,R,Tp,K,PUV,Zc):
        # DE PLANO DE IMAGEN A COORDENADAS NORMALIZADAS DE REFERENCIAL DE CAMARA
        C_norm = np.linalg.inv(K) @ PUV
        # DE COORDENADAS NORMALIZADAS A REFERENCIAL DE CAMARA 
        C = Zc * C_norm
        # AGREGRAR 1 A CORDS DE REFERENCIAL DE CAMARA [Xc;Yc;Zc;1]
        C = np.concatenate((C,[[1]]), axis = 0)
        # CONSTRUIR INVERSA DE LA MATRIZ HOMOGENEA 
        H_inv = np.concatenate((np.concatenate((R.T,(-R.T)@Tp), axis=1), [[0,0,0,1]]), axis=0)
        # DE REFERENCIAL DE CAMARA A REFERENCIAL DE MUNDO
        PW = H_inv @ C
        return PW
    

    def aruOriToWorld(self,R_aru):
        R_aru = cv2.Rodrigues(R_aru)[0]
        # DE REFERENCIAL DE CAMARA A REFRENCIAL DE ARUCO
        R = np.linalg.inv(R_aru) @ self.Rc
        return R