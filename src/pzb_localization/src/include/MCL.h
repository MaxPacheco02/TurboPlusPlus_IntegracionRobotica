#ifndef MCL_H
#define MCL_H

#include "robot.h"

class MCL
{
public:
    MCL(particle* X, command U, 
        double* Z, Point* map); 
    // resample the particles and return the new belief
    particle* resample();

private:
    // map landmarks
    Point* map__;
    int map_N__;
    // robot sensor measurements
    double* Z__;
    int Z_N__;
    // actuation command
    command U__;
    // belief
    particle* X__;
    int X_N__;
    //maximum weight
    double max_weight__;
    // update
    void motion_update();
    // predict
    void sensor_update();
};

#endif /* MCL_H */
