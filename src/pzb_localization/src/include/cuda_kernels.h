#ifndef CUDA_KERNELS_H
#define CUDA_KERNELS_H

#include <cuda_runtime.h>

// Forward declarations to avoid circular dependencies
struct particle;
struct command;
class Point;

// match exactly the extern "C" in your .cu:
extern "C" {
    cudaError_t checkCuda(cudaError_t result);
    void runTest(float *c);
    void particles_motion(particle *X, int X_N, command U);
    void particles_sensing(particle *X, int X_N, double *Z, int Z_N, Point *map, int map_N, double ang_sep);
    void cp_particle_from_to(particle *p_from, particle *p_to, int p_N);
}

#endif // CUDA_KERNELS_H