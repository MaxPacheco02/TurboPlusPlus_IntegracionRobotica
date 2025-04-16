#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <exception>

#include "utility.h"
#include "map_utils.h"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Quaternion.h"

class Robot
{
public:
    Robot();
    
    Robot(geometry_msgs::msg::Pose pose);
    // Set robot's states
    void set_states(double new_x, double new_y, double new_orient);
    // Set robots state transition noise
    void set_noise(double forward_noise, double turn_noise, double sensor_noise);
    // Sense the environment with sensors
    std::vector<double> sense(const std::vector<Point>& map, bool noise = true);
    // Move the robot
    void move(double turn, double forward);
    // Update pose msg
    void update_pose_msg();
    // get sensor noise
    double get_sensor_noise();
    // get x
    double get_x();
    // get y
    double get_y();
    // get pose msg
    geometry_msgs::msg::Pose get_pose_msg();

    double get_theta();
    // Get pose readings
    std::string get_pose();
    // Get sensor readings
    std::string get_sensor_readings(const std::vector<Point>& map);

private:
    // robot states
    geometry_msgs::msg::Pose pose__;

    double x__;
    double y__;
    double orientation__;
    // noise
    static double forward_noise__;
    static double turn_noise__;
    static double sense_noise__;

};

#endif /* ROBOT_H */