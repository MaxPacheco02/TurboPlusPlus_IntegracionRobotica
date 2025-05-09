#!/usr/bin/env python3

# ENCONTRAR PUNTOS EN CUADRO Y OBTENER DISTANCIA ENTRE ELLOS

import cv2
import math
import numpy as np


class POINTS:

    def __init__(self, Wd):

        # PARAMETROS DE CAMERA DE LAPTOP 720p (ESTIMADO)
        self.f = 3
        self.X = 1280
        self.A = 3.6

        # DIMENSIONES DE OBJETO (mm)
        self.Wd = Wd

        # DISTANCIA A CAMARA
        self.d = 0


    def aruDistance(self,PUV):
        Wpx = 0
        w = PUV[1][0] - PUV[0][0]
        h = PUV[1][1] - PUV[0][1]

        Wpx = math.sqrt( (w)**2 + (h)**2 )

        if(Wpx != 0):
            # CALCULO DE DISTANCIA (mm)
            Wmm = (Wpx * self.A) / self.X
            self.d = (self.Wd * self.f) / Wmm
            
        return self.d