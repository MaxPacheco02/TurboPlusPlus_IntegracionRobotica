#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <exception>

#include <eigen3/Eigen/Dense>

#include "utility.h"
#include "map_utils.h"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Quaternion.h"

#include <cuda_runtime.h>

class Robot
{
public:
    Robot();

    // Set robot's states
    void set_states(double new_x, double new_y, double new_orient);
    // Set robots state transition noise
    void set_noise(double forward_noise, double turn_noise, double sensor_noise);
    // Sense the environment with sensors
    __host__ __device__ void sense(const Point *map, int map_N, Point *local_map, int local_map_N,
                                   double *sensor, double ang_sep, geometry_msgs::msg::Pose ext_pose, bool noise);

    __host__ __device__ void transform_measurements(double *meas, double *local_meas, int meas_N, geometry_msgs::msg::Transform tf_cmd, double ang_sep);

    // Move the robot
    __host__ __device__ void move(geometry_msgs::msg::Transform tf_cmd);
    // Update pose msg
    void update_pose_msg();
    // get sensor noise
    __host__ __device__ double get_sensor_noise();
    // get x
    double get_x() const;
    // get y
    double get_y() const;
    // get pose msg
    __host__ __device__ geometry_msgs::msg::Pose get_pose_msg() const;

    double get_theta() const;
    // Get pose readings
    std::string get_pose();

    double x__;
    double y__;
    double orientation__;

private:
    // robot states
    geometry_msgs::msg::Pose pose_msg__;

    // noise
    static double forward_noise__;
    static double turn_noise__;
    static double sense_noise__;
};

#endif /* ROBOT_H */