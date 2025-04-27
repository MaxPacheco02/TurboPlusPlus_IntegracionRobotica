// utility.cpp
#include "utility.h"
#include <cmath>

namespace utility {

double get_gaussian_random_number(double mean, double var) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<double> dist(mean, var);
    return dist(gen);
}

double get_random_number() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}

double get_gaussian_probability(double mean, double var, double x) {
    const double diff = mean - x;
    return exp(-(diff * diff) / (2.0 * var * var)) /
           sqrt(2.0 * M_PI * var * var);
}

__host__ __device__ double get_measurement_prob(
    double* measurement,
    double* sensor,
    int sensor_N,
    double var)
{
    double prob = 1.0;
    
    #ifdef __CUDA_ARCH__
    // Device code path - simplified computation
    for (int i = 0; i < sensor_N; i++) {
        const double diff = measurement[i] - sensor[i];
        prob *= exp(-(diff * diff) / (2.0 * var * var));
    }
    #else
    // Host code path - full computation
    for (int i = 0; i < sensor_N; i++) {
        prob *= get_gaussian_probability(measurement[i], var, sensor[i]);
    }
    #endif

    return prob;
}

} // namespace utility