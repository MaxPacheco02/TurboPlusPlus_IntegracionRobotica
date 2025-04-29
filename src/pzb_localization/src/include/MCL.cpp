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

    // Handle numerical underflow
    if (sum < 1e-50)
    {
        std::cout << "Warning: Weight underflow detected. Resetting weights." << std::endl;
        for (int i = 0; i < num_particles_; i++)
        {
            particles_[i].w = 1.0 / num_particles_;
        }
        max_weight_ = 1.0 / num_particles_;
        return;
    }

    // Calculate effective sample size before normalization
    double neff_before = 1.0 / sum_squared_weights;

    // Normalize weights
    const double inv_sum = 1.0 / sum;
    sum_squared_weights = 0.0;
    for (int i = 0; i < num_particles_; i++)
    {
        particles_[i].w *= inv_sum;
        sum_squared_weights += particles_[i].w * particles_[i].w;
    }

    // Calculate statistics after normalization
    double neff_after = 1.0 / sum_squared_weights;
    double weight_variance = 0.0;
    double mean_weight = 1.0 / num_particles_;

    for (int i = 0; i < num_particles_; i++)
    {
        weight_variance += pow(particles_[i].w - mean_weight, 2);
    }
    weight_variance /= num_particles_;

    // Debug output
    std::cout << "\nWeight normalization statistics:" << std::endl;
    std::cout << "Before normalization:" << std::endl;
    std::cout << "  Min weight: " << min_weight << std::endl;
    std::cout << "  Max weight: " << max_weight_ << std::endl;
    std::cout << "  Sum: " << sum << std::endl;
    std::cout << "  Effective sample size: " << neff_before << std::endl;

    std::cout << "After normalization:" << std::endl;
    std::cout << "  Weight variance: " << weight_variance << std::endl;
    std::cout << "  Effective sample size: " << neff_after << std::endl;

    // Check for degenerate cases
    if (neff_after < num_particles_ * 0.1)
    {
        std::cout << "Warning: Severe particle degeneracy detected!" << std::endl;
    }
}

void MCL::resampleParticles(particle *new_belief)
{
    // Calculate effective sample size
    double sum_squared_weights = 0.0;
    double max_weight = 0.0;
    int best_idx = 0;

    static std::random_device rd;
    static std::mt19937 gen(rd());

    // Calculate statistics
    for (int i = 0; i < num_particles_; i++)
    {
        const double w = particles_[i].w;
        sum_squared_weights += w * w;
        if (w > max_weight)
        {
            max_weight = w;
            best_idx = i;
        }
    }

    double neff = 1.0 / sum_squared_weights;
    // More conservative resampling threshold
    const double resample_threshold = num_particles_ * 0.3; // Increased from 0.7
    bool needs_resampling = (neff > resample_threshold);

    // Calculate cluster statistics to detect convergence
    double mean_x = 0.0, mean_y = 0.0;
    double variance_x = 0.0, variance_y = 0.0;
    
    // First pass: calculate means
    for (int i = 0; i < num_particles_; i++) {
        mean_x += particles_[i].r.get_x() * particles_[i].w;
        mean_y += particles_[i].r.get_y() * particles_[i].w;
    }
    
    // Second pass: calculate variances
    for (int i = 0; i < num_particles_; i++) {
        variance_x += pow(particles_[i].r.get_x() - mean_x, 2) * particles_[i].w;
        variance_y += pow(particles_[i].r.get_y() - mean_y, 2) * particles_[i].w;
    }
    
    // Check if particles are well-clustered (converged)
    double position_variance = variance_x + variance_y;
    bool is_converged = position_variance < 0.1;  // 10cm threshold
    
    std::cout << "Position variance: " << position_variance 
              << " Converged: " << (is_converged ? "Yes" : "No") << std::endl;

    if (needs_resampling)
    {
        std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);

        double r = dist(gen);
        double c = particles_[0].w;
        int i = 0;

        // Reduced noise parameters
        std::normal_distribution<double> pos_noise(0.0, 0.01);   // Reduced from 0.02
        std::normal_distribution<double> angle_noise(0.0, 0.02); // Reduced from 0.05

        for (int m = 0; m < num_particles_; m++)
        {
            double U = r + m * (1.0 / num_particles_);
            while (U > c && i < num_particles_ - 1)
            {
                i++;
                c += particles_[i].w;
            }

            new_belief[m] = particles_[i];

            // Add scaled noise based on motion
            double motion_scale = std::min(1.0,
                                           std::max(fabs(cmd_.forward), fabs(cmd_.turn)) * 2.0);

            double new_x = new_belief[m].r.get_x() + pos_noise(gen) * motion_scale;
            double new_y = new_belief[m].r.get_y() + pos_noise(gen) * motion_scale;
            double new_theta = new_belief[m].r.get_theta() + angle_noise(gen) * motion_scale;

            // Normalize angle
            while (new_theta > M_PI)
                new_theta -= 2.0 * M_PI;
            while (new_theta < -M_PI)
                new_theta += 2.0 * M_PI;

            new_belief[m].r.set_states(new_x, new_y, new_theta);
            new_belief[m].r.set_noise(0.01, 0.02, 0.01); // Reduced noise parameters
            new_belief[m].w = 1.0 / num_particles_;
        }
    }
    else
    {
        // Even smaller noise when not resampling
        std::normal_distribution<double> small_noise(0.0, 0.002); // Reduced from 0.005

        for (int i = 0; i < num_particles_; i++)
        {
            new_belief[i] = particles_[i];

            // Scale noise with motion
            double motion_scale = std::min(1.0,
                                           std::max(fabs(cmd_.forward), fabs(cmd_.turn)) * 2.0);

            double new_x = new_belief[i].r.get_x() + small_noise(gen) * motion_scale;
            double new_y = new_belief[i].r.get_y() + small_noise(gen) * motion_scale;
            double new_theta = new_belief[i].r.get_theta() + small_noise(gen) * motion_scale;

            new_belief[i].r.set_states(new_x, new_y, new_theta);
        }
    }
}

// This goes into the resampling function. Test later
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

double MCL::findMaxWeight() const
{
    return max_weight_;
}