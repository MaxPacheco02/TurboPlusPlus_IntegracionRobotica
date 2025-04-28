#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <stdio.h>
#include "robot.h"
#include "robot_utils.h"
#include "utility.h"
#include "map_utils.h"

extern cudaError_t checkCuda(cudaError_t result);

// Constants moved to constant memory for faster access
__constant__ float d_noise_params[3]; // forward_noise, turn_noise, sense_noise
__constant__ int d_grid_params[2];    // number of rays, measurements per thread

struct DeviceParticle
{
    float x, y, theta, weight;
};

// Forward declarations of kernel functions
__global__ void setupRandomStates(curandState *states, unsigned long seed, int N);
__global__ void raycastKernel(const Point *map, int map_size, DeviceParticle *particles,
                              float *measurements, int num_rays, double angle_sep);
__global__ void moveParticlesKernel(DeviceParticle *particles, int N, float turn,
                                    float forward, curandState *states);
__global__ void calculateWeightsKernel(DeviceParticle *particles, float *measurements,
                                       float *actual_measurements, int N, int num_rays);

class CUDAMemoryManager
{
private:
    // Device pointers
    DeviceParticle *d_particles = nullptr;
    curandState *d_random_states = nullptr;
    float *d_weights = nullptr;
    Point *d_map = nullptr;
    int d_map_size = 0;

    // Pinned host memory for efficient transfers
    DeviceParticle *h_particles = nullptr;
    float *h_weights = nullptr;

public:
    ~CUDAMemoryManager() { freeResources(); }

    bool initialize(int particles_N, Point *map_data, int map_size)
    {
        // Allocate device memory
        if (cudaMalloc(&d_particles, particles_N * sizeof(DeviceParticle)) != cudaSuccess ||
            cudaMalloc(&d_random_states, particles_N * sizeof(curandState)) != cudaSuccess ||
            cudaMalloc(&d_weights, particles_N * sizeof(float)) != cudaSuccess ||
            cudaMalloc(&d_map, map_size * sizeof(Point)) != cudaSuccess)
        {
            freeResources();
            return false;
        }

        // Allocate pinned host memory
        if (cudaMallocHost(&h_particles, particles_N * sizeof(DeviceParticle)) != cudaSuccess ||
            cudaMallocHost(&h_weights, particles_N * sizeof(float)) != cudaSuccess)
        {
            freeResources();
            return false;
        }

        // Copy map data to device (done once)
        if (cudaMemcpy(d_map, map_data, map_size * sizeof(Point),
                       cudaMemcpyHostToDevice) != cudaSuccess)
        {
            freeResources();
            return false;
        }
        d_map_size = map_size;

        // Initialize random states
        const int threadsPerBlock = 256;
        const int blocks = (particles_N + threadsPerBlock - 1) / threadsPerBlock;
        setupRandomStates<<<blocks, threadsPerBlock>>>(d_random_states, time(NULL), particles_N);

        // Initialize noise parameters in constant memory
        float h_noise_params[3] = {0.05f, 0.01f, 0.1f}; // forward, turn, sense noise
        cudaMemcpyToSymbol(d_noise_params, h_noise_params, sizeof(float) * 3);

        return cudaGetLastError() == cudaSuccess;
    }

    // Efficient batch update of particles
    bool updateParticles(particle *host_particles, int N)
    {
        // Convert host particles to device format
        for (int i = 0; i < N; i++)
        {
            h_particles[i].x = host_particles[i].r.get_x();
            h_particles[i].y = host_particles[i].r.get_y();
            h_particles[i].theta = host_particles[i].r.get_theta();
            h_particles[i].weight = host_particles[i].w;
        }

        // Async copy to device
        return cudaMemcpyAsync(d_particles, h_particles, N * sizeof(DeviceParticle),
                               cudaMemcpyHostToDevice) == cudaSuccess;
    }

    // Efficient batch retrieval of particles
    bool retrieveParticles(particle *host_particles, int N)
    {
        // Async copy from device
        if (cudaMemcpyAsync(h_particles, d_particles, N * sizeof(DeviceParticle),
                            cudaMemcpyDeviceToHost) != cudaSuccess)
        {
            return false;
        }
        cudaDeviceSynchronize();

        // Convert back to host format
        for (int i = 0; i < N; i++)
        {
            host_particles[i].r.set_states(h_particles[i].x, h_particles[i].y, h_particles[i].theta);
            host_particles[i].w = h_particles[i].weight;
        }
        return true;
    }

    // Accessors
    DeviceParticle *particles() const { return d_particles; }
    Point *map() const { return d_map; }
    int mapSize() const { return d_map_size; }
    curandState *randomStates() const { return d_random_states; }
    float *weights() const { return d_weights; }

private:
    void freeResources()
    {
        if (d_particles)
            cudaFree(d_particles);
        if (d_random_states)
            cudaFree(d_random_states);
        if (d_weights)
            cudaFree(d_weights);
        if (d_map)
            cudaFree(d_map);
        if (h_particles)
            cudaFreeHost(h_particles);
        if (h_weights)
            cudaFreeHost(h_weights);

        d_particles = nullptr;
        d_random_states = nullptr;
        d_weights = nullptr;
        d_map = nullptr;
        h_particles = nullptr;
        h_weights = nullptr;
    }
};

// Global memory manager instance
static CUDAMemoryManager g_memoryManager;

// Kernel implementations

__device__ double calculateMeasurementLikelihood(
    double predicted, 
    double actual, 
    double var) 
{
    const double diff = predicted - actual;
    const double squared_diff = diff * diff;
    
    // More robust likelihood function with bounded minimum
    const double min_likelihood = 1e-3;
    const double max_squared_diff = 8.0 * var;  // Allow larger differences
    
    double likelihood = exp(-squared_diff / max_squared_diff);
    return fmax(likelihood, min_likelihood);
}

__global__ void calculateWeightsKernel(
    DeviceParticle* particles,
    float* measurements,
    float* actual_measurements,
    int N,
    int num_rays)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N) return;

    float total_likelihood = 1.0f;
    const float measurement_var = d_noise_params[2] * d_noise_params[2];
    int valid_measurements = 0;
    
    // Get particle's orientation
    const float particle_theta = particles[idx].theta;
    
    // Calculate number of rays to shift based on particle orientation
    const float angle_per_ray = (2.0f * M_PI) / num_rays;
    const int ray_shift = static_cast<int>(particle_theta / angle_per_ray + 0.5f);
    
    // Use fewer measurements to reduce correlation
    const int measurement_stride = 4;  // Reduced from 8 for better orientation sensitivity
    for (int j = 0; j < num_rays; j += measurement_stride) {
        // Get index with orientation shift
        int shifted_idx = (j + ray_shift) % num_rays;
        float predicted = measurements[idx * num_rays + j];
        float actual = actual_measurements[shifted_idx];
        
        if (predicted > 0.0f && actual > 0.0f) {
            float likelihood = calculateMeasurementLikelihood(
                predicted, actual, measurement_var);
            total_likelihood *= likelihood;
            valid_measurements++;
        }
    }

    // Calculate final weight
    if (valid_measurements > 0) {
        // Take nth root to prevent numerical underflow
        const float power = 1.0f / (valid_measurements * 2.0f);
        particles[idx].weight = powf(total_likelihood, power);
    } else {
        particles[idx].weight = 1e-3f;  // Minimum weight for invalid measurements
    }
}

__global__ void setupRandomStates(curandState *states, unsigned long seed, int N)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < N)
    {
        curand_init(seed, idx, 0, &states[idx]);
    }
}

__global__ void raycastKernel(
    const Point *map,
    int map_size,
    DeviceParticle *particles,
    float *measurements,
    int num_rays,
    double angle_sep)
{
    const int particle_idx = blockIdx.x;
    const int ray_idx = threadIdx.x;

    if (ray_idx >= num_rays)
        return;

    const float particle_x = particles[particle_idx].x;
    const float particle_y = particles[particle_idx].y;
    const float particle_theta = particles[particle_idx].theta;

    // Calculate ray angle relative to particle's orientation
    const float ray_angle = particle_theta + ray_idx * angle_sep;  // Start from particle's orientation
    float min_dist = INFINITY;

    // Use shared memory for temporary calculations
    __shared__ float distances[256]; // Assuming max threads per block = 256

    // Each thread processes its own ray
    for (int i = 0; i < map_size; i++)
    {
        float dx = map[i].x - particle_x;
        float dy = map[i].y - particle_y;
        float dist = sqrtf(dx * dx + dy * dy);

        float angle = atan2f(dy, dx);
        
        // Normalize angle difference to [-π, π]
        float angle_diff = angle - ray_angle;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        angle_diff = fabsf(angle_diff);

        if (angle_diff < 0.1f && dist < min_dist)
        {
            min_dist = dist;
        }
    }

    distances[ray_idx] = min_dist;
    __syncthreads();

    // First thread writes measurements
    if (ray_idx == 0)
    {
        for (int i = 0; i < num_rays; i++)
        {
            measurements[particle_idx * num_rays + i] = distances[i];
        }
    }
}

__global__ void moveParticlesKernel(
    DeviceParticle* particles,
    int N,
    float turn,
    float forward,
    curandState* states)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N) return;

    // Get current particle state
    float x = particles[idx].x;
    float y = particles[idx].y;
    float theta = particles[idx].theta;

    // Calculate noisy control inputs
    float turn_noise = curand_normal(&states[idx]) * d_noise_params[1];
    float forward_noise = curand_normal(&states[idx]) * d_noise_params[0];
    
    // Apply turn first (with noise)
    float noisy_turn = turn + turn_noise;
    theta += noisy_turn;
    
    // Normalize theta to [-π, π]
    while (theta > M_PI) theta -= 2 * M_PI;
    while (theta < -M_PI) theta += 2 * M_PI;
    
    // Apply forward motion in the new direction (with noise)
    float noisy_forward = forward + forward_noise;
    x += noisy_forward * cosf(theta);
    y += noisy_forward * sinf(theta);
    
    // Add small random walk for diversity (prevents particle depletion)
    const float diversity_scale = 0.001f;
    x += curand_normal(&states[idx]) * diversity_scale;
    y += curand_normal(&states[idx]) * diversity_scale;
    theta += curand_normal(&states[idx]) * diversity_scale;

    // Store updated state
    particles[idx].x = x;
    particles[idx].y = y;
    particles[idx].theta = theta;
}

// Host interface implementation
extern "C"
{

    bool initializeMCL(int particles_N, Point *map_data, int map_N)
    {
        return g_memoryManager.initialize(particles_N, map_data, map_N);
    }

    void particlesMotion(particle *X, int N, command U)
    {
        // Update device particles
        g_memoryManager.updateParticles(X, N);

        // Launch movement kernel
        const int threadsPerBlock = 256;
        const int blocks = (N + threadsPerBlock - 1) / threadsPerBlock;

        moveParticlesKernel<<<blocks, threadsPerBlock>>>(
            g_memoryManager.particles(),
            N,
            U.turn,
            U.forward,
            g_memoryManager.randomStates());

        // Retrieve updated particles
        g_memoryManager.retrieveParticles(X, N);
    }

    void particlesSensing(
        particle *X,
        int X_N,
        double *Z,
        int Z_N,
        Point *map,
        int map_N,
        double ang_sep)
    {
        // Update device particles
        g_memoryManager.updateParticles(X, X_N);

        // Allocate device memory for measurements
        float *d_measurements;
        float *d_actual_measurements;
        cudaMalloc(&d_measurements, X_N * Z_N * sizeof(float));
        cudaMalloc(&d_actual_measurements, Z_N * sizeof(float));

        // Copy actual measurements to device
        float *h_actual_measurements = new float[Z_N];
        for (int i = 0; i < Z_N; i++)
        {
            h_actual_measurements[i] = static_cast<float>(Z[i]);
        }
        cudaMemcpy(d_actual_measurements, h_actual_measurements,
                   Z_N * sizeof(float), cudaMemcpyHostToDevice);
        delete[] h_actual_measurements;

        // Launch raycast kernel
        raycastKernel<<<X_N, Z_N>>>(
            g_memoryManager.map(),
            g_memoryManager.mapSize(),
            g_memoryManager.particles(),
            d_measurements,
            Z_N,
            ang_sep * M_PI / 180.0);

        // Launch weight calculation kernel
        const int threadsPerBlock = 256;
        const int blocks = (X_N + threadsPerBlock - 1) / threadsPerBlock;

        calculateWeightsKernel<<<blocks, threadsPerBlock>>>(
            g_memoryManager.particles(),
            d_measurements,
            d_actual_measurements,
            X_N,
            Z_N);

        // Check for errors
        cudaError_t err = cudaGetLastError();
        if (err != cudaSuccess)
        {
            fprintf(stderr, "CUDA Error: %s\n", cudaGetErrorString(err));
        }

        // Cleanup
        cudaFree(d_measurements);
        cudaFree(d_actual_measurements);

        // Retrieve updated particles
        g_memoryManager.retrieveParticles(X, X_N);
    }
} // extern "C"