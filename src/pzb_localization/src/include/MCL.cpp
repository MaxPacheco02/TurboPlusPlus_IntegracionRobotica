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

void MCL::resampleParticles(particle *new_belief)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // Calculate mean pose before resampling
    double mean_x = 0.0, mean_y = 0.0;
    double mean_cos = 0.0, mean_sin = 0.0;
    double total_weight = 0.0;

    for (int i = 0; i < num_particles_; i++)
    {
        const double w = particles_[i].w;
        mean_x += particles_[i].r.get_x() * w;
        mean_y += particles_[i].r.get_y() * w;
        mean_cos += std::cos(particles_[i].r.get_theta()) * w;
        mean_sin += std::sin(particles_[i].r.get_theta()) * w;
        total_weight += w;
    }

    mean_x /= total_weight;
    mean_y /= total_weight;
    double mean_theta = std::atan2(mean_sin, mean_cos);

    // Determine if robot is stationary based on control commands
    bool is_stationary = (std::abs(cmd_.forward) < 0.0001 && std::abs(cmd_.turn) < 0.001);

    // Adjust noise based on motion state
    double pos_noise = is_stationary ? 0.001 : 0.02;   // Reduced noise when stationary
    double angle_noise = is_stationary ? 0.001 : 0.05; // Very small angle noise when stationary

    // Initialize resampling wheel
    int index = static_cast<int>(dist(gen) * num_particles_);
    double beta = 0.0;
    const double step = 2.0 * max_weight_ / num_particles_;

    std::normal_distribution<double> pos_noise_dist(0.0, pos_noise);
    std::normal_distribution<double> angle_noise_dist(0.0, angle_noise);

    int resampled_count = 0;
    for (int i = 0; i < num_particles_; i++)
    {
        beta += dist(gen) * step;

        while (beta > particles_[index].w)
        {
            beta -= particles_[index].w;
            index = (index + 1) % num_particles_;
            resampled_count++;
        }

        // Copy particle
        new_belief[i] = particles_[index];

        // Add noise based on motion state
        if (is_stationary)
        {
            // Very conservative updates when stationary
            double noise_scale = std::min(1.0, particles_[index].w / max_weight_);
            new_belief[i].r.set_states(
                new_belief[i].r.get_x() + pos_noise_dist(gen) * noise_scale,
                new_belief[i].r.get_y() + pos_noise_dist(gen) * noise_scale,
                new_belief[i].r.get_theta() + angle_noise_dist(gen) * noise_scale);
        }
        else
        {
            // Normal noise when moving
            new_belief[i].r.set_states(
                new_belief[i].r.get_x() + pos_noise_dist(gen),
                new_belief[i].r.get_y() + pos_noise_dist(gen),
                new_belief[i].r.get_theta() + angle_noise_dist(gen));
        }

        // Reset weight
        new_belief[i].w = 1.0 / num_particles_;
    }

    std::cout << "Resampling iterations: " << resampled_count << std::endl;

    // When stationary, ensure some particles maintain the previous mean orientation
    if (is_stationary)
    {
        const int num_anchor_particles = num_particles_ / 10; // 10% of particles
        for (int i = 0; i < num_anchor_particles; i++)
        {
            int idx = static_cast<int>(dist(gen) * num_particles_);
            new_belief[idx].r.set_states(
                new_belief[idx].r.get_x(),
                new_belief[idx].r.get_y(),
                mean_theta + angle_noise_dist(gen) * 0.1 // Very small noise
            );
        }
    }
    // // Add some random particles with probability 0.05 (for the future, it helps in kidnapping problem)
    // for (int i = 0; i < num_particles_; ++i)
    // {
    //     if (utility::get_random_number() < 0.05)
    //     { // 5% chance of random particle
    //         // Create a random particle near the current estimated position
    //         double mean_x = 0.0, mean_y = 0.0;
    //         for (int j = 0; j < num_particles_; j++)
    //         {
    //             mean_x += particles_[j].r.get_x() / num_particles_;
    //             mean_y += particles_[j].r.get_y() / num_particles_;
    //         }

    //         Robot r;
    //         r.set_states(
    //             mean_x + utility::get_gaussian_random_number(0.0, 1.0),
    //             mean_y + utility::get_gaussian_random_number(0.0, 1.0),
    //             utility::get_random_number() * 2.0 * M_PI);
    //         // r.set_noise(noise_fwd, noise_fwd, noise_fwd);
    //         r.set_noise(0.05, 0.05, 0.05);
    //         new_belief[i] = particle{r, 0.05 * max_weight_}; // Give it a reasonable weight
    //     }
    // }
}

double MCL::findMaxWeight() const
{
    return max_weight_;
}