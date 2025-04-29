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

struct DeviceParticle;

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
    const double max_squared_diff = 8.0 * var; // Allow larger differences

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

    float total_log_likelihood = 0.0f;
    const float measurement_var = 0.08f;  // Might need tuning
    int valid_measurements = 0;
    
    for (int j = 0; j < num_rays; j += 2) {
        const float predicted = measurements[idx * num_rays + j];
        const float actual = actual_measurements[j];
        
        if (predicted > 0.1f && actual > 0.1f) {
            const float diff = predicted - actual;
            const float squared_diff = diff * diff;
            
            // More aggressive likelihood function
            float log_likelihood = -squared_diff / (2.0f * measurement_var);
            total_log_likelihood += log_likelihood;
            valid_measurements++;
        }
    }
    
    // Calculate final weight
    float weight;
    if (valid_measurements > 5) {
        // Convert from log space and apply exponential scaling
        weight = expf(total_log_likelihood * 5.5f);  // Multiply by 2 to make differences more pronounced
        
        // Ensure reasonable bounds
        const float min_weight = 1e-6f;
        const float max_weight = 1.0f;
        weight = fmaxf(fminf(weight, max_weight), min_weight);
    } else {
        weight = 1e-6f;  // Very low weight for particles with few valid measurements
    }
    
    particles[idx].weight = weight;
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
    float ray_angle = particle_theta + ray_idx * angle_sep;

    // Normalize angle to [-π, π]
    while (ray_angle > M_PI)
        ray_angle -= 2.0f * M_PI;
    while (ray_angle < -M_PI)
        ray_angle += 2.0f * M_PI;

    float min_dist = INFINITY;
    int closest_idx = -1;

    // Use shared memory for temporary calculations
    __shared__ float distances[256]; // Assuming max threads per block = 256

    // Each thread processes its own ray
    for (int i = 0; i < map_size; i++)
    {
        float dx = map[i].x - particle_x;
        float dy = map[i].y - particle_y;
        float dist = sqrtf(dx * dx + dy * dy);

        float point_angle = atan2f(dy, dx);

        // Normalize angle difference to [-π, π]
        float angle_diff = point_angle - ray_angle;
        while (angle_diff > M_PI)
            angle_diff -= 2.0f * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2.0f * M_PI;
        angle_diff = fabsf(angle_diff);

        const float angle_threshold = 0.1f * angle_sep;
        if (angle_diff < angle_threshold && dist < min_dist)
        {
            min_dist = dist;
            closest_idx = i;
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
    DeviceParticle *particles,
    int N,
    float turn,
    float forward,
    curandState *states)
{
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N)
        return;

    // Get current particle state
    float x = particles[idx].x;
    float y = particles[idx].y;
    float theta = particles[idx].theta;

    // Only apply motion if there is actual movement
    if (fabs(forward) > 1e-6 || fabs(turn) > 1e-6)
    {
        // Calculate noisy control inputs - scale noise with motion magnitude
        float turn_noise = curand_normal(&states[idx]) * d_noise_params[1] * fmaxf(0.1f, fabs(turn));
        float forward_noise = curand_normal(&states[idx]) * d_noise_params[0] * fmaxf(0.1f, forward);

        // Apply turn first (with noise)
        float noisy_turn = turn + turn_noise;
        theta += noisy_turn;

        // Normalize theta to [-π, π]
        while (theta > M_PI)
            theta -= 2 * M_PI;
        while (theta < -M_PI)
            theta += 2 * M_PI;

        // Apply forward motion in the new direction (with noise)
        float noisy_forward = forward + forward_noise;
        x += noisy_forward * cosf(theta);
        y += noisy_forward * sinf(theta);

        // Add minimal diversity noise
        const float min_noise = 0.0001f;
        x += curand_normal(&states[idx]) * min_noise;
        y += curand_normal(&states[idx]) * min_noise;
        theta += curand_normal(&states[idx]) * min_noise;
    }

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
        particle* X, int X_N,
        double* Z, int Z_N,
        Point* map, int map_N,
        double ang_sep)
    {
        // Print actual measurements for debugging
        std::cout << "\nActual measurements (first 5):" << std::endl;
        for (int i = 0; i < std::min(5, Z_N); i++) {
            std::cout << "Z[" << i << "] = " << Z[i] << std::endl;
        }
    
        // Allocate device memory
        float* d_measurements;
        float* d_actual_measurements;
        cudaMalloc(&d_measurements, X_N * Z_N * sizeof(float));
        cudaMalloc(&d_actual_measurements, Z_N * sizeof(float));
    
        // Convert actual measurements to float
        float* h_actual_measurements = new float[Z_N];
        for (int i = 0; i < Z_N; i++) {
            h_actual_measurements[i] = static_cast<float>(Z[i]);
        }
    
        // Copy to device
        cudaMemcpy(d_actual_measurements, h_actual_measurements,
                   Z_N * sizeof(float), cudaMemcpyHostToDevice);
    
        // Convert particles
        DeviceParticle* d_particles;
        cudaMalloc(&d_particles, X_N * sizeof(DeviceParticle));
        DeviceParticle* h_particles = new DeviceParticle[X_N];
        
        for (int i = 0; i < X_N; i++) {
            h_particles[i].x = X[i].r.get_x();
            h_particles[i].y = X[i].r.get_y();
            h_particles[i].theta = X[i].r.get_theta();
            h_particles[i].weight = X[i].w;
        }
        
        cudaMemcpy(d_particles, h_particles,
                   X_N * sizeof(DeviceParticle), cudaMemcpyHostToDevice);
    
        // First get predicted measurements
        raycastKernel<<<X_N, Z_N>>>(
            map, map_N,
            d_particles,
            d_measurements,
            Z_N,
            ang_sep * M_PI / 180.0);
    
        // Then calculate weights
        dim3 blockDim(256);
        dim3 gridDim((X_N + blockDim.x - 1) / blockDim.x);
    
        calculateWeightsKernel<<<gridDim, blockDim>>>(
            d_particles,
            d_measurements,
            d_actual_measurements,
            X_N,
            Z_N);
    
        // Copy back results
        cudaMemcpy(h_particles, d_particles,
                   X_N * sizeof(DeviceParticle), cudaMemcpyDeviceToHost);
    
        // Convert back to host particles
        for (int i = 0; i < X_N; i++) {
            X[i].r.set_states(h_particles[i].x, h_particles[i].y, h_particles[i].theta);
            X[i].w = h_particles[i].weight;
        }
    
        // Print some debug info
        std::cout << "\nWeight distribution after update:" << std::endl;
        float min_w = 1.0f, max_w = 0.0f, sum_w = 0.0f;
        for (int i = 0; i < X_N; i++) {
            min_w = fmin(min_w, X[i].w);
            max_w = fmax(max_w, X[i].w);
            sum_w += X[i].w;
        }
        std::cout << "Min weight: " << min_w << std::endl;
        std::cout << "Max weight: " << max_w << std::endl;
        std::cout << "Average weight: " << sum_w/X_N << std::endl;
    
        // Cleanup
        delete[] h_actual_measurements;
        delete[] h_particles;
        cudaFree(d_measurements);
        cudaFree(d_actual_measurements);
        cudaFree(d_particles);
    }


} // extern "C"