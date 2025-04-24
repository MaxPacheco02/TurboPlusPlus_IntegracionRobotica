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

struct particle
{
    particle(Robot r, double w);
    Robot r;
    double w;
};

struct command
{
    command();
    command(double f, double t);
    double forward;
    double turn;
};

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
    void sense(const Point *map, double *sensor, int map_N, double ang_sep, geometry_msgs::msg::Pose ext_pose, bool noise);
    // Move the robot
    void move(double turn, double forward);
    // Update pose msg
    void update_pose_msg();
    // get sensor noise
    double get_sensor_noise();
    // get x
    double get_x() const;
    // get y
    double get_y() const;
    // get pose msg
    geometry_msgs::msg::Pose get_pose_msg() const;

    double get_theta() const;
    // Get pose readings
    std::string get_pose();

    const static int tpb__{1024};

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