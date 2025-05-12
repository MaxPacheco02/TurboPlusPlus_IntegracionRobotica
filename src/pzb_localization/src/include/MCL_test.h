#pragma once

#include "map_utils.h"
#include "robot.h"
#include <vector>

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

class MCL {
public:
    MCL(const std::vector<particle>& init_particles,
              const std::vector<Point>& map_points,
              const std::vector<double>& ranges = {},
              double angle_separation = 9.0,
              const command& cmd = command());

    std::vector<particle> resample();
    void set_motion(double forward, double turn);
    void set_measurement(const std::vector<double>& ranges);

private:
    void motion_update();
    void sensor_update();
    void normalize_weights();
    double get_measurement_prob(const std::vector<double>& sensed);
    double find_max_weight() const;

    std::vector<particle> X__;
    std::vector<Point> map__;
    std::vector<double> Z__;
    command U__;
    double ang_sep__ = 9.0;
    double max_weight__ = 0.0;
};
