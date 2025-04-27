#include "cuda_kernels.h"
#include <cstdio>

cudaError_t checkCuda(cudaError_t result) {
    if (result != cudaSuccess) {
        fprintf(stderr, "CUDA Error: %s\n", cudaGetErrorString(result));
    }
    return result;
}