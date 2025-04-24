#pragma once
#include <cuda_runtime.h>

// match exactly the extern "C" in your .cu:
extern "C" {
cudaError_t checkCuda(cudaError_t result);
void initWith(float num, float *a, int N);
void launchAdd(float *c, float *a, float *b, int N);
bool checkElementsAre(float target, float *array, int N);
void runTest(float *c);
}