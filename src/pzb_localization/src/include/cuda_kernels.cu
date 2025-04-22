// src/cuda_kernels.cu
#include <stdio.h>
#include <assert.h>
#include <cuda_runtime.h>

extern "C" cudaError_t checkCuda(cudaError_t result)
{
  if (result != cudaSuccess)
  {
    fprintf(stderr, "CUDA Runtime Error: %s\n", cudaGetErrorString(result));
    assert(result == cudaSuccess);
  }
  // else
  // {
  //   fprintf(stderr, "CUDA Runtime Success!\n");
  // }
  return result;
}

extern "C" void initWith(float num, float *a, int N)
{
  for (int i = 0; i < N; ++i)
    a[i] = num;
}

extern "C" bool checkElementsAre(float target, float *array, int N)
{
  for (int i = 0; i < N; i++)
  {
    if (array[i] != target)
    {
      printf("FAIL: array[%d] = %0.0f != %0.0f\n", i, array[i], target);
      return false;
    }
  }
  printf("SUCCESS! All values added correctly.\n");
  return true;
}

__global__ void addVectorsInto(float *result, float *a, float *b, int N)
{
  int i = threadIdx.x + blockIdx.x * blockDim.x;
  if (i < N)
    result[i] = a[i] + b[i];
}

extern "C" void launchAdd(float *c, float *a, float *b, int N)
{
  const int threads_per_block = 1024;
  const int number_of_blocks = (N + threads_per_block - 1) / threads_per_block;

  addVectorsInto<<<number_of_blocks, threads_per_block>>>(c, a, b, N);

  checkCuda(cudaDeviceSynchronize());
}

__global__ void testKernel(float *c) {
  c[0] = 123.0f;
}

extern "C" void runTest(float *c) {
  testKernel<<<1,1>>>(c);
  cudaDeviceSynchronize();
}