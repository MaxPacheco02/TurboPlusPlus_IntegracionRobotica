// utility.h
#ifndef UTILITY_H
#define UTILITY_H

#include <cuda_runtime.h>
#include <random>

namespace utility {

/**
 * Random number generation functions
 */
double get_gaussian_random_number(double mean, double var);
double get_random_number();

/**
 * Probability computation functions
 */
double get_gaussian_probability(double mean, double var, double x);

__host__ __device__ double get_measurement_prob(
    double* measurement,
    double* sensor,
    int sensor_N,
    double var);

} // namespace utility

#endif // UTILITY_H