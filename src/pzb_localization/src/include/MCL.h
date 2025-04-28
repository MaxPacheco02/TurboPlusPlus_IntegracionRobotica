#ifndef MCL_H
#define MCL_H

#include "robot_utils.h"
#include <assert.h>
#include "cuda_kernels.h"

class MCL
{
public:
    MCL(particle *X, int X_N, geometry_msgs::msg::Transform tf_cmd, double *Z,
        int Z_N, Point *map, int map_N, double ang_sep);
    // resample the particles and return the new belief
    void resample(particle *new_belief, double noise_fwd, double noise_turn, double noise_sensor);

private:
    // map landmarks
    Point *map__;
    int map_N__;
    // robot sensor measurements
    double *Z__;
    int Z_N__;
    // actuation command
    geometry_msgs::msg::Transform tf_cmd__;
    // belief
    particle *X__;
    int X_N__;
    // maximum weight
    double max_weight__;
    // LiDAR separation angle
    double ang_sep__;
    // update
    void motion_update();
    // predict
    void sensor_update();
};

#endif /* MCL_H */
