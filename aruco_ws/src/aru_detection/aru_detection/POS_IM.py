#!/usr/bin/env python3

# MOVING AVERAGE/LOW PASS PARA POSICION Y ORIENTACION DE ARUCO

from collections import deque

class POINTS:

    def __init__(self,n):

        # PARA MOVING AVERGAE
        # self.n = n
        # self.queue = deque(maxlen=self.n)

        # PARA LOWPASS
        self.x_esstA = 0
        self.alpha = 0.87

    # MOVING AVERAGE
    # def ma_get(self,x):
    #     self.queue.append(x)
    #     x_bar = sum(self.queue)/len(self.queue)
    #     return x_bar
    
    # LOWPASS
    def lp_get(self,x):
        x_est = self.alpha*self.x_esstA + (1-self.alpha)*x
        self.x_esstA = x_est
        return x_est