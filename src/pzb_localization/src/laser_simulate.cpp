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
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class LaserSimulate : public rclcpp::Node
{
public:
    LaserSimulate() : Node("laser_simulate")
    {
        using namespace std::placeholders;

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10,
            [this](const nav_msgs::msg::OccupancyGrid &msg)
            {
                map_ = msg;
                map_received_ = true;
                RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f", 
                           map_.info.width, map_.info.height, map_.info.resolution);
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            [this](const nav_msgs::msg::Odometry &msg)
            {
                if (map_received_) {
                    // Compute raycast from map at odom's pose
                    computeRaycast(msg);
                }
            });

        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/fwd_avoidance_lidar", 10);
        
        // Declare lidar parameters
        this->declare_parameter("laser_angle_min", -M_PI);          // Start angle (radians)
        this->declare_parameter("laser_angle_max", M_PI);           // End angle (radians)
        // this->declare_parameter("laser_angle_increment", M_PI/180 * 18); // Angular resolution
        this->declare_parameter("laser_angle_increment", M_PI/180 * 5); // Angular resolution (1 degree)
        this->declare_parameter("laser_range_min", 0.1);           // Minimum range (meters)
        this->declare_parameter("laser_range_max", 1.0);           // Maximum range (meters)
        this->declare_parameter("laser_frame_id", "base_axis");     // Frame ID
        this->declare_parameter("occupied_threshold", 65);          // Occupancy threshold (0-100)
        this->declare_parameter("unknown_threshold", -1);           // Unknown cell threshold
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    sensor_msgs::msg::LaserScan laser_msg_;
    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;

    void computeRaycast(const nav_msgs::msg::Odometry& odom_msg)
    {
        // Get laser parameters
        double angle_min = this->get_parameter("laser_angle_min").as_double();
        double angle_max = this->get_parameter("laser_angle_max").as_double();
        double angle_increment = this->get_parameter("laser_angle_increment").as_double();
        double range_min = this->get_parameter("laser_range_min").as_double();
        double range_max = this->get_parameter("laser_range_max").as_double();
        std::string frame_id = this->get_parameter("laser_frame_id").as_string();
        int occupied_threshold = this->get_parameter("occupied_threshold").as_int();
        int unknown_threshold = this->get_parameter("unknown_threshold").as_int();

        // Extract robot pose
        double robot_x = odom_msg.pose.pose.position.x;
        double robot_y = odom_msg.pose.pose.position.y;
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Setup laser scan message
        laser_msg_.header.stamp = this->get_clock()->now();
        laser_msg_.header.frame_id = frame_id;
        laser_msg_.angle_min = angle_min;
        laser_msg_.angle_max = angle_max;
        laser_msg_.angle_increment = angle_increment;
        laser_msg_.time_increment = 0.0;
        laser_msg_.scan_time = 0.1; // 10 Hz
        laser_msg_.range_min = range_min;
        laser_msg_.range_max = range_max;

        // Calculate number of rays
        int num_rays = static_cast<int>((angle_max - angle_min) / angle_increment) + 1;
        laser_msg_.ranges.clear();
        laser_msg_.ranges.resize(num_rays);
        laser_msg_.intensities.clear();
        laser_msg_.intensities.resize(num_rays);

        // Cast rays
        for (int i = 0; i < num_rays; ++i) {
            double ray_angle = angle_min + i * angle_increment;
            double global_angle = yaw + ray_angle;
            
            double range = castRay(robot_x, robot_y, global_angle, range_max, occupied_threshold, unknown_threshold);
            
            // Clamp range to valid bounds
            if (range < range_min) {
                range = range_min;
            } else if (range > range_max) {
                range = range_max;
            }
            
            laser_msg_.ranges[i] = range;
            laser_msg_.intensities[i] = 0.0; // Not used in this simulation
        }

        // Publish the laser scan
        laser_pub_->publish(laser_msg_);
    }

    double castRay(double start_x, double start_y, double angle, double max_range, 
                   int occupied_threshold, int unknown_threshold)
    {
        // Ray direction
        double dx = std::cos(angle);
        double dy = std::sin(angle);
        
        // Step size for ray marching (smaller = more accurate, larger = faster)
        double step_size = map_.info.resolution * 0.5;
        double current_range = 0.0;
        
        while (current_range < max_range) {
            // Current position along the ray
            double current_x = start_x + current_range * dx;
            double current_y = start_y + current_range * dy;
            
            // Convert world coordinates to grid coordinates
            int grid_x, grid_y;
            if (!worldToGrid(current_x, current_y, grid_x, grid_y)) {
                // Outside map bounds
                break;
            }
            
            // Get occupancy value at this grid cell
            int occupancy = getOccupancyValue(grid_x, grid_y);
            
            // Check if we hit an obstacle
            if (occupancy >= occupied_threshold) {
                return current_range;
            }
            
            // Optional: Stop at unknown cells
            if (unknown_threshold >= 0 && occupancy == -1) {
                return current_range;
            }
            
            // Move to next step
            current_range += step_size;
        }
        
        // No obstacle found within max range
        return max_range;
    }

    bool worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y)
    {
        // Convert world coordinates to grid coordinates
        grid_x = static_cast<int>((world_x - map_.info.origin.position.x) / map_.info.resolution);
        grid_y = static_cast<int>((world_y - map_.info.origin.position.y) / map_.info.resolution);
        
        // Check bounds
        return (grid_x >= 0 && grid_x < static_cast<int>(map_.info.width) && 
                grid_y >= 0 && grid_y < static_cast<int>(map_.info.height));
    }

    int getOccupancyValue(int grid_x, int grid_y)
    {
        // Check bounds
        if (grid_x < 0 || grid_x >= static_cast<int>(map_.info.width) || 
            grid_y < 0 || grid_y >= static_cast<int>(map_.info.height)) {
            return 100; // Treat out-of-bounds as occupied
        }
        
        // Get value from occupancy grid
        int index = grid_y * map_.info.width + grid_x;
        return map_.data[index];
    }

    void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y)
    {
        world_x = grid_x * map_.info.resolution + map_.info.origin.position.x;
        world_y = grid_y * map_.info.resolution + map_.info.origin.position.y;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserSimulate>());
    rclcpp::shutdown();
    return 0;
}