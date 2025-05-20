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

    resampleParticles(new_belief);
}

void MCL::motionUpdate()
{
    particlesMotion(particles_, num_particles_, cmd_);
}

void MCL::sensorUpdate()
{
    particlesSensing(particles_, num_particles_, measurements_,
                     num_measurements_, map_, num_map_points_, angle_sep_);
}

void MCL::normalizeWeights()
{
    double sum = 0.0;
    max_weight_ = 0.0;
    double min_weight = 1.0;
    double sum_squared_weights = 0.0;

    // First pass: calculate statistics
    for (int i = 0; i < num_particles_; i++)
    {
        double w = particles_[i].w;
        sum += w;
        sum_squared_weights += w * w;
        max_weight_ = std::max(max_weight_, w);
        min_weight = std::min(min_weight, w);
    }

    // Only normalize if we have valid weights
    if (max_weight_ > 1e-10)
    {
        // Scale weights to sum to 1 while preserving ratios
        const double scale = 1.0 / sum;
        for (int i = 0; i < num_particles_; i++)
        {
            particles_[i].w *= scale;
        }
    }
    else
    {
        // If all weights are too small, reset to uniform
        for (int i = 0; i < num_particles_; i++)
        {
            particles_[i].w = 1.0 / num_particles_;
        }
        max_weight_ = 1.0 / num_particles_;
    }
}

void MCL::resampleParticles(particle* new_belief)
{
    // Calculate effective sample size and position variance
    double sum_squared_weights = 0.0;
    double weighted_x = 0.0, weighted_y = 0.0;
    double total_weight = 0.0;

    // First pass: calculate weighted mean position
    for (int i = 0; i < num_particles_; i++) {
        double w = particles_[i].w;
        weighted_x += particles_[i].r.get_x() * w;
        weighted_y += particles_[i].r.get_y() * w;
        total_weight += w;
        sum_squared_weights += w * w;
    }

    double mean_x = weighted_x / total_weight;
    double mean_y = weighted_y / total_weight;

    // Calculate position variance
    double position_variance = 0.0;
    for (int i = 0; i < num_particles_; i++) {
        double dx = particles_[i].r.get_x() - mean_x;
        double dy = particles_[i].r.get_y() - mean_y;
        position_variance += (dx * dx + dy * dy) * particles_[i].w;
    }
    position_variance /= total_weight;

    // Calculate effective sample size
    double neff = 1.0 / sum_squared_weights;

    // Now use position_variance for adaptive resampling
    const double resample_threshold = num_particles_ * (position_variance < 0.1 ? 0.7 : 0.5);
    bool needs_resampling = (neff < resample_threshold);

    if (needs_resampling)
    {
        std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
        double r = dist(gen);
        double c = particles_[0].w;
        int i = 0;

        // Calculate adaptive noise based on uncertainty
        double adapt_noise = std::max(0.01, std::min(0.1, position_variance * 0.2));
        std::normal_distribution<double> pos_noise(0.0, adapt_noise);
        std::normal_distribution<double> angle_noise(0.0, adapt_noise * 2.0);

        for (int m = 0; m < num_particles_; m++)
        {
            double U = r + m * (1.0 / num_particles_);
            while (U > c && i < num_particles_ - 1)
            {
                i++;
                c += particles_[i].w;
            }

            new_belief[m] = particles_[i];

            // Add adaptive noise
            double new_x = new_belief[m].r.get_x() + pos_noise(gen);
            double new_y = new_belief[m].r.get_y() + pos_noise(gen);
            double new_theta = new_belief[m].r.get_theta() + angle_noise(gen);

            // Normalize angle
            while (new_theta > M_PI) new_theta -= 2.0 * M_PI;
            while (new_theta < -M_PI) new_theta += 2.0 * M_PI;

            new_belief[m].r.set_states(new_x, new_y, new_theta);
        }
    }
    else
    {
        // Just copy particles if not resampling
        for (int i = 0; i < num_particles_; i++)
        {
            new_belief[i] = particles_[i];
        }
    }
}

double MCL::findMaxWeight() const
{
    return max_weight_;
}