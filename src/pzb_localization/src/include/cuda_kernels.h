#ifndef CUDA_KERNELS_H
#define CUDA_KERNELS_H

#include <cuda_runtime.h>
#include "robot.h"
#include "robot_utils.h"
#include "map_utils.h"

extern "C" {
    /**
     * Initialize MCL CUDA resources
     * @param particles_N Number of particles
     * @param map_data Pointer to map data
     * @param map_N Number of map points
     * @return true if initialization successful
     */
    bool initializeMCL(int particles_N, Point* map_data, int map_N);

    /**
     * Update particle positions based on motion model
     * @param X Array of particles
     * @param X_N Number of particles
     * @param U Control command
     */
    void particlesMotion(particle* X, int X_N, command U);

    /**
     * Update particle weights based on sensor measurements
     * @param X Array of particles
     * @param X_N Number of particles
     * @param Z Array of measurements
     * @param Z_N Number of measurements
     * @param map Array of map points
     * @param map_N Number of map points
     * @param ang_sep Angular separation between rays
     */
    void particlesSensing(particle* X, int X_N, double* Z, int Z_N, 
                         Point* map, int map_N, double ang_sep);

    /**
     * Check CUDA errors
     * @param result CUDA error code
     * @return Same error code
     */
    cudaError_t checkCuda(cudaError_t result);
}

#endif // CUDA_KERNELS_H