
// robot_utils.h
#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

#include "robot.h"

/**
 * Particle structure for particle filter
 */
struct particle {
    particle(Robot r = Robot(), double w = 0.0) : r(r), w(w) {}
    Robot r;
    double w;
};

/**
 * Command structure for robot motion
 */
struct command {
    command(double f = 0.0, double t = 0.0) : forward(f), turn(t) {}
    double forward;
    double turn;
};

#endif // ROBOT_UTILS_H