#include "robot_utils.h"

particle::particle(Robot rr, double ww) : r(rr), w(ww) {}

command::command(double f, double t) : forward(f), turn(t) {}
command::command() {}