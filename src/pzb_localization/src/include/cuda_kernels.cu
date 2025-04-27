#include <cuda_runtime.h>
#include "robot.h"
#include "robot_utils.h"
#include "utility.h"
#include "map_utils.h"

// Simple device-only structs - no ROS dependencies
struct DevicePoint {
    float x;
    float y;
};

// For device code only - simplified versions of your functions
__device__ void device_move(float &x, float &y, float &orientation, 
                           float turn, float forward)
{
    orientation = fmodf((orientation + turn), (2.0f * 3.14159f));
    x += forward * cosf(orientation);
    y += forward * sinf(orientation);
}

// Device kernels that operate on simplified data
__global__ void device_move_particles(float *x, float *y, float *orientation, int N, 
                                    float turn, float forward)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i < N) {
        device_move(x[i], y[i], orientation[i], turn, forward);
    }
}

// Device function to calculate distance between two points
__device__ float device_distance(float x1, float y1, float x2, float y2) {
    return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// Device function for Gaussian probability
__device__ float device_gaussian_probability(float mean, float var, float x) {
    return expf(-(powf((mean - x), 2)) / (powf(var, 2)) / 2.0f) / sqrtf(2.0f * 3.14159f * (powf(var, 2)));
}

// Device function to calculate measurement probability
__device__ float device_measurement_prob(float* measurement, float* sensor, int sensor_N, float var) {
    float prob = 1.0f;
    for(int i = 0; i < sensor_N; i++) {
        prob *= device_gaussian_probability(measurement[i], var, sensor[i]);
    }
    return prob;
}

// Device kernel for sensing
__global__ void device_sense_particles(float *x, float *y, float *orientation, float *weights,
                                     float *measurements, int measurements_N,
                                     float *map_x, float *map_y, int map_N,
                                     float sensor_noise, int particle_N)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i < particle_N) {
        // Simple measurement model for device code
        float particle_measurements[40]; // Assuming max 40 measurements
        
        // In a real implementation, you would calculate measurements based on the map
        // Here we're using a simplified approach
        for (int j = 0; j < measurements_N; j++) {
            // Find closest map point for this measurement angle
            float min_dist = 1000000.0f;
            for (int k = 0; k < map_N; k++) {
                float dist = device_distance(x[i], y[i], map_x[k], map_y[k]);
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
            particle_measurements[j] = min_dist;
        }
        
        // Calculate measurement probability
        weights[i] = device_measurement_prob(particle_measurements, measurements, measurements_N, sensor_noise);
    }
}

// Copy particles kernel
__global__ void device_copy_particles(float *x_from, float *y_from, float *orientation_from, float *w_from,
                                    float *x_to, float *y_to, float *orientation_to, float *w_to, 
                                    int N)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if (i < N) {
        x_to[i] = x_from[i];
        y_to[i] = y_from[i];
        orientation_to[i] = orientation_from[i];
        w_to[i] = w_from[i];
    }
}

// Minimal error checking
extern "C" cudaError_t checkCuda(cudaError_t result)
{
    if (result != cudaSuccess) {
        fprintf(stderr, "CUDA Runtime Error: %s\n", cudaGetErrorString(result));
    }
    return result;
}

// Wrapper functions that prepare data for device, call kernel, then copy back
extern "C" void particles_motion(particle *X, int X_N, command U)
{
    // Allocate device memory
    float *d_x, *d_y, *d_orientation;
    cudaMalloc(&d_x, X_N * sizeof(float));
    cudaMalloc(&d_y, X_N * sizeof(float));
    cudaMalloc(&d_orientation, X_N * sizeof(float));
    
    // Copy data to device
    float *h_x = new float[X_N];
    float *h_y = new float[X_N];
    float *h_orientation = new float[X_N];
    
    for (int i = 0; i < X_N; i++) {
        h_x[i] = X[i].r.get_x();
        h_y[i] = X[i].r.get_y();
        h_orientation[i] = X[i].r.get_theta();
    }
    
    cudaMemcpy(d_x, h_x, X_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, h_y, X_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_orientation, h_orientation, X_N * sizeof(float), cudaMemcpyHostToDevice);
    
    // Launch kernel
    int blockSize = 256;
    int numBlocks = (X_N + blockSize - 1) / blockSize;
    device_move_particles<<<numBlocks, blockSize>>>(d_x, d_y, d_orientation, X_N, U.turn, U.forward);
    
    // Copy results back
    cudaMemcpy(h_x, d_x, X_N * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_y, d_y, X_N * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_orientation, d_orientation, X_N * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Update original particles
    for (int i = 0; i < X_N; i++) {
        X[i].r.set_states(h_x[i], h_y[i], h_orientation[i]);
    }
    
    // Free memory
    delete[] h_x;
    delete[] h_y;
    delete[] h_orientation;
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_orientation);
}

extern "C" void particles_sensing(particle *X, int X_N, double *Z, int Z_N, Point *map, int map_N, double ang_sep)
{
    // Allocate host memory
    float *h_x = new float[X_N];
    float *h_y = new float[X_N];
    float *h_orientation = new float[X_N];
    float *h_weights = new float[X_N];
    
    float *h_map_x = new float[map_N];
    float *h_map_y = new float[map_N];
    float *h_measurements = new float[Z_N];
    
    // Fill host arrays
    for (int i = 0; i < X_N; i++) {
        h_x[i] = X[i].r.get_x();
        h_y[i] = X[i].r.get_y();
        h_orientation[i] = X[i].r.get_theta();
        h_weights[i] = X[i].w;
    }
    
    for (int i = 0; i < map_N; i++) {
        h_map_x[i] = map[i].x;
        h_map_y[i] = map[i].y;
    }
    
    for (int i = 0; i < Z_N; i++) {
        h_measurements[i] = Z[i];
    }
    
    // Allocate device memory
    float *d_x, *d_y, *d_orientation, *d_weights;
    float *d_map_x, *d_map_y, *d_measurements;
    
    cudaMalloc(&d_x, X_N * sizeof(float));
    cudaMalloc(&d_y, X_N * sizeof(float));
    cudaMalloc(&d_orientation, X_N * sizeof(float));
    cudaMalloc(&d_weights, X_N * sizeof(float));
    
    cudaMalloc(&d_map_x, map_N * sizeof(float));
    cudaMalloc(&d_map_y, map_N * sizeof(float));
    cudaMalloc(&d_measurements, Z_N * sizeof(float));
    
    // Copy to device
    cudaMemcpy(d_x, h_x, X_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, h_y, X_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_orientation, h_orientation, X_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_weights, h_weights, X_N * sizeof(float), cudaMemcpyHostToDevice);
    
    cudaMemcpy(d_map_x, h_map_x, map_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_map_y, h_map_y, map_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_measurements, h_measurements, Z_N * sizeof(float), cudaMemcpyHostToDevice);
    
    // Get sensor noise from first particle (all particles have the same sensor noise)
    float sensor_noise = X[0].r.get_sensor_noise();
    
    // Launch kernel
    int blockSize = 256;
    int numBlocks = (X_N + blockSize - 1) / blockSize;
    device_sense_particles<<<numBlocks, blockSize>>>(
        d_x, d_y, d_orientation, d_weights,
        d_measurements, Z_N,
        d_map_x, d_map_y, map_N,
        sensor_noise, X_N
    );
    
    // Copy results back
    cudaMemcpy(h_weights, d_weights, X_N * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Update original particles
    for (int i = 0; i < X_N; i++) {
        X[i].w = h_weights[i];
    }
    
    // Free memory
    delete[] h_x;
    delete[] h_y;
    delete[] h_orientation;
    delete[] h_weights;
    delete[] h_map_x;
    delete[] h_map_y;
    delete[] h_measurements;
    
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_orientation);
    cudaFree(d_weights);
    cudaFree(d_map_x);
    cudaFree(d_map_y);
    cudaFree(d_measurements);
}

extern "C" void cp_particle_from_to(particle *p_from, particle *p_to, int p_N)
{
    // Allocate device memory
    float *d_x_from, *d_y_from, *d_orientation_from, *d_w_from;
    float *d_x_to, *d_y_to, *d_orientation_to, *d_w_to;
    
    cudaMalloc(&d_x_from, p_N * sizeof(float));
    cudaMalloc(&d_y_from, p_N * sizeof(float));
    cudaMalloc(&d_orientation_from, p_N * sizeof(float));
    cudaMalloc(&d_w_from, p_N * sizeof(float));
    
    cudaMalloc(&d_x_to, p_N * sizeof(float));
    cudaMalloc(&d_y_to, p_N * sizeof(float));
    cudaMalloc(&d_orientation_to, p_N * sizeof(float));
    cudaMalloc(&d_w_to, p_N * sizeof(float));
    
    // Copy data to device
    float *h_x_from = new float[p_N];
    float *h_y_from = new float[p_N];
    float *h_orientation_from = new float[p_N];
    float *h_w_from = new float[p_N];
    
    for (int i = 0; i < p_N; i++) {
        h_x_from[i] = p_from[i].r.get_x();
        h_y_from[i] = p_from[i].r.get_y();
        h_orientation_from[i] = p_from[i].r.get_theta();
        h_w_from[i] = p_from[i].w;
    }
    
    cudaMemcpy(d_x_from, h_x_from, p_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y_from, h_y_from, p_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_orientation_from, h_orientation_from, p_N * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_w_from, h_w_from, p_N * sizeof(float), cudaMemcpyHostToDevice);
    
    // Launch kernel to copy on device
    int blockSize = 256;
    int numBlocks = (p_N + blockSize - 1) / blockSize;
    device_copy_particles<<<numBlocks, blockSize>>>(
        d_x_from, d_y_from, d_orientation_from, d_w_from,
        d_x_to, d_y_to, d_orientation_to, d_w_to,
        p_N
    );
    checkCuda(cudaDeviceSynchronize());
    
    // Copy results back
    float *h_x_to = new float[p_N];
    float *h_y_to = new float[p_N];
    float *h_orientation_to = new float[p_N];
    float *h_w_to = new float[p_N];
    
    cudaMemcpy(h_x_to, d_x_to, p_N * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_y_to, d_y_to, p_N * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_orientation_to, d_orientation_to, p_N * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_w_to, d_w_to, p_N * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Update destination particles
    for (int i = 0; i < p_N; i++) {
        p_to[i].r.set_states(h_x_to[i], h_y_to[i], h_orientation_to[i]);
        p_to[i].w = h_w_to[i];
    }
    
    // Free memory
    delete[] h_x_from;
    delete[] h_y_from;
    delete[] h_orientation_from;
    delete[] h_w_from;
    
    delete[] h_x_to;
    delete[] h_y_to;
    delete[] h_orientation_to;
    delete[] h_w_to;
    
    cudaFree(d_x_from);
    cudaFree(d_y_from);
    cudaFree(d_orientation_from);
    cudaFree(d_w_from);
    
    cudaFree(d_x_to);
    cudaFree(d_y_to);
    cudaFree(d_orientation_to);
    cudaFree(d_w_to);
}

// Function for initialization
extern "C" void initWith(float a, float *x, int N)
{
    for(int i = 0; i < N; ++i)
    {
        x[i] = a;
    }
}

// Simple CUDA add function (for testing)
__global__ void addVectorsInto(float *result, float *a, float *b, int N)
{
    int i = threadIdx.x + blockIdx.x * blockDim.x;
    if(i < N)
    {
        result[i] = a[i] + b[i];
    }
}

// Test function wrapper
extern "C" void launchAdd(float *c, float *a, float *b, int N)
{
    int blockSize = 256;
    int numBlocks = (N + blockSize - 1) / blockSize;
    addVectorsInto<<<numBlocks, blockSize>>>(c, a, b, N);
}

// Check results function
extern "C" void checkElementsAre(float target, float *array, int N)
{
    for(int i = 0; i < N; i++)
    {
        if(array[i] != target)
        {
            printf("FAIL: array[%d] - %0.0f does not equal %0.0f\n", i, array[i], target);
            exit(1);
        }
    }
    printf("SUCCESS! All values match %0.0f\n", target);
}

// Simple test function that was in your cuda_kernels.h
extern "C" void runTest(float *c)
{
    const int N = 100;
    size_t size = N * sizeof(float);
    
    float *a;
    float *b;
    
    checkCuda(cudaMallocManaged(&a, size));
    checkCuda(cudaMallocManaged(&b, size));
    checkCuda(cudaMallocManaged(&c, size));
    
    initWith(3, a, N);
    initWith(4, b, N);
    initWith(0, c, N);
    
    launchAdd(c, a, b, N);
    
    checkCuda(cudaGetLastError());
    checkCuda(cudaDeviceSynchronize());
    
    checkElementsAre(7, c, N);
    
    checkCuda(cudaFree(a));
    checkCuda(cudaFree(b));
    // Note: c is not freed here since it's passed in
}