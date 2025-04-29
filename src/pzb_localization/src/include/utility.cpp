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
    double log_prob = 0.0;
    const double denominator = 2.0 * var * var;
    
    // Only consider a subset of measurements for efficiency and robustness
    const int max_rays_to_consider = std::min(40, sensor_N);
    const int step = sensor_N / max_rays_to_consider;
    
    int count = 0;
    for (int i = 0; i < sensor_N; i += std::max(1, step)) {
        if (count >= max_rays_to_consider) break;
        
        // Skip invalid measurements (too large or too small)
        if (std::isnan(sensor[i]) || std::isnan(measurement[i]) || 
            measurement[i] < 0.01 || sensor[i] < 0.01) {
            continue;
        }
            
        const double diff = measurement[i] - sensor[i];
        log_prob += -(diff * diff) / denominator;
        count++;
    }
    
    // Convert back from log space
    return exp(log_prob);
}

} // namespace utility