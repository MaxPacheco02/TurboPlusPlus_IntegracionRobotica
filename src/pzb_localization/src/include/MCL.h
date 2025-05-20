#ifndef MCL_H
#define MCL_H

#include "robot_utils.h"
#include "cuda_kernels.h"
#include <random>

/**
 * Monte Carlo Localization implementation
 */
class MCL {
public:
    MCL(particle* X, int X_N, command U, double* Z, 
        int Z_N, Point* map, int map_N, double ang_sep);

    void resample(particle* new_belief);

private:
    void motionUpdate();
    void sensorUpdate();
    void resampleParticles(particle* new_belief);
    void normalizeWeights();
    double findMaxWeight() const;

    std::mt19937 gen{std::random_device{}()};

    particle* particles_;
    int num_particles_;
    command cmd_;
    double* measurements_;
    int num_measurements_;
    Point* map_;
    int num_map_points_;
    double angle_sep_;
    double max_weight_;
};

#endif // MCL_H