#include "MCL_test.h"
#include "utility.h"
#include <cmath>
#include <numeric>
#include <algorithm>

MCL::MCL(const std::vector<particle>& init_particles,
               const std::vector<Point>& map_points,
               const std::vector<double>& ranges,
               double angle_separation,
               const command& cmd)
{
    X__ = init_particles;
    map__ = map_points;
    Z__ = ranges;
    ang_sep__ = angle_separation;
    U__ = cmd;
}

void MCL::set_motion(double forward, double turn) {
    U__.forward = forward;
    U__.turn = turn;
}

void MCL::set_measurement(const std::vector<double>& ranges) {
    Z__ = ranges;
}

void MCL::motion_update()
{
    for (auto& p : X__) {
        p.r.move(U__.turn, U__.forward);

        double motion_scale = std::min(1.0, std::max(std::abs(U__.forward), std::abs(U__.turn)) * 2.0);
        double dx = utility::get_gaussian_random_number(0.0, 0.01 * motion_scale);
        double dy = utility::get_gaussian_random_number(0.0, 0.01 * motion_scale);
        double dtheta = utility::get_gaussian_random_number(0.0, 0.02 * motion_scale);

        p.r.set_states(p.r.get_x() + dx, p.r.get_y() + dy, p.r.get_theta() + dtheta);
    }
}

void MCL::sensor_update()
{
    max_weight__ = 0.0;

    for (auto& p : X__) {
        std::vector<Point> local_map;
        std::vector<double> sensed;

        auto pose = p.r.get_pose_msg();
        p.r.sense_cpu(map__, local_map, sensed, pose, ang_sep__, false);  // simulated sense

        p.w = get_measurement_prob(sensed);
        max_weight__ = std::max(max_weight__, p.w);
    }
}

double MCL::get_measurement_prob(const std::vector<double>& sensed)
{
    double prob = 1.0;
    const double sigma = 0.1;

    for (size_t i = 0; i < std::min(Z__.size(), sensed.size()); ++i) {
        double diff = Z__[i] - sensed[i];
        double weight = std::exp(-0.5 * diff * diff / (sigma * sigma));
        prob *= weight;
    }

    return prob;
}

void MCL::normalize_weights()
{
    double sum = 0.0;
    for (const auto& p : X__) sum += p.w;

    if (sum < 1e-50) {
        for (auto& p : X__) p.w = 1.0 / X__.size();
        max_weight__ = 1.0 / X__.size();
        return;
    }

    for (auto& p : X__) p.w /= sum;
}

double MCL::find_max_weight() const {
    return max_weight__;
}

std::vector<particle> MCL::resample()
{
    motion_update();
    sensor_update();
    normalize_weights();

    double sum_sq = 0.0;
    for (const auto& p : X__) sum_sq += p.w * p.w;
    double neff = 1.0 / sum_sq;

    bool should_resample = neff < (0.5 * X__.size());
    std::vector<particle> new_particles = X__;

    if (should_resample) {
        int index = utility::get_random_number() * X__.size();
        double beta = 0.0;

        for (auto& p : new_particles) {
            beta += utility::get_random_number() * 2.0 * max_weight__;
            while (beta > X__[index].w) {
                beta -= X__[index].w;
                index = (index + 1) % X__.size();
            }

            p = X__[index];

            // Resampling noise
            p.r.set_states(
                p.r.get_x() + utility::get_gaussian_random_number(0.0, 0.01),
                p.r.get_y() + utility::get_gaussian_random_number(0.0, 0.01),
                p.r.get_theta() + utility::get_gaussian_random_number(0.0, 0.02));

            p.w = 1.0 / X__.size();
        }
    }

    return new_particles;
}
