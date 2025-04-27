// MCL.cpp
#include "MCL.h"
#include <algorithm>
#include <random>
#include <iostream>
#include <numeric>
#include <cmath>

MCL::MCL(particle *X, int X_N, command U, double *Z, int Z_N,
         Point *map, int map_N, double ang_sep)
    : particles_(X), num_particles_(X_N), cmd_(U), measurements_(Z),
      num_measurements_(Z_N), map_(map), num_map_points_(map_N),
      angle_sep_(ang_sep), max_weight_(0.0)
{
    if (!initializeMCL(X_N, map, map_N))
    {
        throw std::runtime_error("Failed to initialize CUDA resources");
    }
}

void MCL::resample(particle *new_belief)
{
    motionUpdate();
    sensorUpdate();
    normalizeWeights();

    // Debug weight statistics
    double sum_weights = 0.0;
    double sum_squared_weights = 0.0;
    double min_weight = 1.0;
    double max_weight = 0.0;

    for (int i = 0; i < num_particles_; i++)
    {
        double w = particles_[i].w;
        sum_weights += w;
        sum_squared_weights += w * w;
        min_weight = std::min(min_weight, w);
        max_weight = std::max(max_weight, w);
    }

    // Calculate effective sample size
    double neff = 1.0 / sum_squared_weights;

    std::cout << "\nParticle Filter Statistics:" << std::endl;
    std::cout << "Min weight: " << min_weight << std::endl;
    std::cout << "Max weight: " << max_weight << std::endl;
    std::cout << "Weight variance: " << sum_squared_weights - (sum_weights * sum_weights / num_particles_) << std::endl;
    std::cout << "Effective sample size: " << neff << " (" << (neff / num_particles_ * 100) << "%)" << std::endl;

    // Sample positions for first 5 particles before resampling
    std::cout << "\nBefore resampling (first 5 particles):" << std::endl;
    for (int i = 0; i < std::min(5, num_particles_); i++)
    {
        std::cout << "Particle " << i << ": x=" << particles_[i].r.get_x()
                  << ", y=" << particles_[i].r.get_y()
                  << ", theta=" << particles_[i].r.get_theta()
                  << ", w=" << particles_[i].w << std::endl;
    }

    resampleParticles(new_belief);

    // Sample positions after resampling
    std::cout << "\nAfter resampling (first 5 particles):" << std::endl;
    for (int i = 0; i < std::min(5, num_particles_); i++)
    {
        std::cout << "Particle " << i << ": x=" << new_belief[i].r.get_x()
                  << ", y=" << new_belief[i].r.get_y()
                  << ", theta=" << new_belief[i].r.get_theta()
                  << ", w=" << new_belief[i].w << std::endl;
    }
}

void MCL::motionUpdate()
{
    particlesMotion(particles_, num_particles_, cmd_);
}

void MCL::sensorUpdate()
{
    // Debug prints
    std::cout << "\nSensor measurements (first 5):" << std::endl;
    for (int i = 0; i < std::min(5, num_measurements_); i++)
    {
        std::cout << "Z[" << i << "] = " << measurements_[i] << std::endl;
    }

    particlesSensing(particles_, num_particles_, measurements_,
                     num_measurements_, map_, num_map_points_, angle_sep_);

    // Debug weights after sensing
    std::cout << "\nParticle weights after sensing (first 5):" << std::endl;
    for (int i = 0; i < std::min(5, num_particles_); i++)
    {
        std::cout << "w[" << i << "] = " << particles_[i].w << std::endl;
    }
}

void MCL::normalizeWeights()
{
    double sum = 0.0;
    max_weight_ = 0.0;

    // Find sum and maximum
    for (int i = 0; i < num_particles_; i++)
    {
        sum += particles_[i].w;
        max_weight_ = std::max(max_weight_, particles_[i].w);
    }

    // Normalize if sum is not zero
    if (sum > 0.0)
    {
        for (int i = 0; i < num_particles_; i++)
        {
            particles_[i].w /= sum;
        }
    }
    else
    {
        // If all weights are zero, set to uniform distribution
        double uniform_weight = 1.0 / num_particles_;
        for (int i = 0; i < num_particles_; i++)
        {
            particles_[i].w = uniform_weight;
        }
        max_weight_ = uniform_weight;
    }

    // Debug normalized weights
    std::cout << "\nNormalized weights (first 5):" << std::endl;
    for (int i = 0; i < std::min(5, num_particles_); i++)
    {
        std::cout << "normalized w[" << i << "] = " << particles_[i].w << std::endl;
    }
}

void MCL::resampleParticles(particle* new_belief) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::normal_distribution<double> pos_noise(0.0, 0.02);  // Small position noise
    std::normal_distribution<double> angle_noise(0.0, 0.05);  // Small angle noise

    // Initialize resampling wheel
    int index = static_cast<int>(dist(gen) * num_particles_);
    double beta = 0.0;
    const double step = 2.0 * max_weight_ / num_particles_;

    int resampled_count = 0;
    for (int i = 0; i < num_particles_; i++) {
        beta += dist(gen) * step;
        
        while (beta > particles_[index].w) {
            beta -= particles_[index].w;
            index = (index + 1) % num_particles_;
            resampled_count++;
        }
        
        // Copy particle and add noise
        new_belief[i] = particles_[index];
        new_belief[i].r.set_states(
            new_belief[i].r.get_x() + pos_noise(gen),
            new_belief[i].r.get_y() + pos_noise(gen),
            new_belief[i].r.get_theta() + angle_noise(gen)
        );
        new_belief[i].w = 1.0 / num_particles_;
    }

    std::cout << "Resampling iterations: " << resampled_count << std::endl;
}

double MCL::findMaxWeight() const
{
    return max_weight_;
}