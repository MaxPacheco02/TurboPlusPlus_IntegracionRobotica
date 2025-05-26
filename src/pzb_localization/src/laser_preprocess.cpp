#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <queue>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class LaserPreprocess : public rclcpp::Node
{
public:
    LaserPreprocess() : Node("laser_preprocess")
    {
        using namespace std::placeholders;

        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](const sensor_msgs::msg::LaserScan &msg)
            {
                laser_msg.ranges.clear();
                laser_msg.header = msg.header;
                laser_msg.scan_time = msg.scan_time;
                laser_msg.time_increment = msg.time_increment;

                // If we want 720 points:
                int DESIRED_POINTS = 360;
                const double stride = static_cast<double>(msg.ranges.size()) / DESIRED_POINTS;

                double ang_min{0}, ang_max{0};
                double range_min{0}, range_max{0};

                laser_msg.ranges.reserve(DESIRED_POINTS);
                laser_msg.intensities.reserve(DESIRED_POINTS);

                // ang = min + inc * i
                // If i want it at zero...
                // i = (0 - min) / inc
                int anchor_zero = - msg.angle_min / msg.angle_increment;
                
                for (size_t i = 0; i < DESIRED_POINTS; i ++)
                {
                    // std::size_t idx = static_cast<std::size_t>(std::round(i * stride) + msg.ranges.size() / 2 + anchor_zero) % msg.ranges.size();
                    std::size_t idx = static_cast<std::size_t>(std::round(i * stride)) % msg.ranges.size();

                    laser_msg.ranges.push_back(msg.ranges[idx]);
                    laser_msg.intensities.push_back(msg.intensities[idx]);

                    double current_ang = msg.angle_min + idx * msg.angle_increment;
                    if (current_ang < ang_min)
                        ang_min = current_ang;
                    if (current_ang > ang_max)
                        ang_max = current_ang;
                }

                laser_msg.angle_min = 0;
                laser_msg.angle_max = 2 * M_PI;
                laser_msg.angle_increment = (ang_max - ang_min) / laser_msg.ranges.size();
                
                laser_msg.range_min = 0.0;
                laser_msg.range_max = 1.0;

                laser_pub_->publish(laser_msg);
            });

        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/lidar", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;

    sensor_msgs::msg::LaserScan laser_msg;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserPreprocess>());
    rclcpp::shutdown();
    return 0;
}
