#ifndef ROBOT_H
#define ROBOT_H

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cuda_runtime.h>
#include "map_utils.h"
#include "utility.h"

/**
 * Robot class representing the physical robot's state and behaviors
 */
class Robot {
public:
    Robot();
    
    // State management
    void set_states(double new_x, double new_y, double new_orient);
    void set_noise(double forward_noise, double turn_noise, double sensor_noise);
    
    // Motion and sensing
    __host__ __device__ void move(double turn, double forward);
    __host__ __device__ void sense(
        const Point* map, int map_N,
        Point* local_map, int local_map_N,
        double* sensor, double ang_sep,
        geometry_msgs::msg::Pose ext_pose,
        bool noise);

    // State access
    __host__ __device__ double get_x() const { return x__; }
    __host__ __device__ double get_y() const { return y__; }
    __host__ __device__ double get_theta() const { return orientation__; }
    __host__ __device__ double get_sensor_noise() const { return sense_noise__; }
    __host__ __device__ geometry_msgs::msg::Pose get_pose_msg() const { return pose__; }
    std::string get_pose() const;

    // Constants
    static constexpr int THREADS_PER_BLOCK = 256;

private:
    // State variables
    double x__;
    double y__;
    double orientation__;
    geometry_msgs::msg::Pose pose__;

    // Noise parameters
    static double forward_noise__;
    static double turn_noise__;
    static double sense_noise__;

    // Internal methods
    void update_pose_msg();
};

#endif /* ROBOT_H */