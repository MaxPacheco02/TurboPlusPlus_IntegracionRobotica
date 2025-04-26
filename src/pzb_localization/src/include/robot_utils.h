#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

#include "robot.h"

struct particle
{
    particle(Robot r, double w);
    Robot r;
    double w;
};

struct command
{
    command();
    command(double f, double t);
    double forward;
    double turn;
};

#endif // ROBOT_UTILS_H
