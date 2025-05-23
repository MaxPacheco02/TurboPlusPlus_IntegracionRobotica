#!/usr/bin/env python3

# MOVING AVERAGE/LOW PASS PARA POSICION Y ORIENTACION DE ARUCO

from collections import deque

class POINTS:

    def __init__(self,n):

        # PARA MOVING AVERGAE
        self.n = n
        self.queue = [deque(maxlen=self.n) for _ in range(7)]

        # PARA LOWPASS
        self.x_esstA = 0
        self.alpha = 0.5

    # MOVING AVERAGE
    def ma_get(self,x,num_state_var): 
        self.queue[num_state_var].append(x)
        x_bar = sum(self.queue[num_state_var])/len(self.queue[num_state_var])
        return x_bar
    
    # LOWPASS
    def lp_get(self,x):
        x_est = self.alpha*self.x_esstA + (1-self.alpha)*x
        self.x_esstA = x_est
        return x_est