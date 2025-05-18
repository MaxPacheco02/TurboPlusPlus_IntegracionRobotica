#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <queue>
#include <iostream>
#include <vector>
#include <optional>

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
#include "tf2/utils.hpp"

using namespace std::chrono_literals;

class ReactiveNavigationNode : public rclcpp::Node
{
public:
    ReactiveNavigationNode() : Node("reactive_navigation_node")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", 10,
            [this](const sensor_msgs::msg::LaserScan &msg)
            {
                angle_min = msg.angle_min;
                angle_max = msg.angle_max;

                // Reduce skip_rays to get better coverage
                int skip_rays = static_cast<int>((scan_filter_angle * M_PI / 180.0) / msg.angle_increment);
                skip_rays = std::max(1, std::min(skip_rays, 10)); // Cap at 10 to ensure sufficient coverage
                angle_increment = msg.angle_increment * skip_rays;

                laserscan_vector.clear();
                scan_angles.clear();

                for (size_t i = 0; i < msg.ranges.size(); i += skip_rays)
                {
                    double angle = msg.angle_min + i * msg.angle_increment;
                    angle = normalize_angle(angle);

                    if (std::isfinite(msg.ranges[i]) &&
                        msg.ranges[i] >= msg.range_min &&
                        msg.ranges[i] <= msg.range_max)
                    {
                        laserscan_vector.push_back(msg.ranges[i]);
                        scan_angles.push_back(angle);
                    }
                }

                RCLCPP_INFO(this->get_logger(), "Processed %zu valid points", laserscan_vector.size());
                for (size_t i = 0; i < std::min(size_t(5), laserscan_vector.size()); ++i)
                {
                    RCLCPP_INFO(this->get_logger(), "Point %zu: range=%.3f, angle=%.1f°",
                                i, laserscan_vector[i], scan_angles[i] * 180.0 / M_PI);
                }
            });

        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            [this](const nav_msgs::msg::Odometry &msg)
            {
                pose_ << msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    tf2::getYaw(msg.pose.pose.orientation);

                double theta = pose_(2);
                rotM_ << std::cos(theta), -std::sin(theta), 0,
                    std::sin(theta), std::cos(theta), 0,
                    0, 0, 1;
            });

        nav_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "nav/goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped &msg)
            {
                received_goal = true;

                goal_pose_ << msg.pose.position.x,
                    msg.pose.position.y,
                    tf2::getYaw(msg.pose.orientation);
            });

        pid_ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        update_timer_ = this->create_wall_timer(10ms, std::bind(&ReactiveNavigationNode::update, this));
        pid_ref_msg.header.frame_id = "odom";
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pid_ref_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    tf2::Quaternion quat;
    geometry_msgs::msg::PoseStamped pid_ref_msg;

    double scan_filter_angle{10.0};
    std::vector<double> laserscan_vector;
    std::vector<double> scan_angles; // Store actual angles for each point
    double angle_min, angle_max, angle_increment;

    Eigen::Vector3f pose_, goal_pose_;
    Eigen::Matrix3f rotM_;

    double received_goal{false};

    void update()
    {
        if (!received_goal)
            return;

        pid_ref_msg.header.stamp = this->get_clock()->now();

        // Transform goal position to robot's local frame
        Eigen::Vector2f goal_global(goal_pose_(0) - pose_(0),
                                    goal_pose_(1) - pose_(1));

        // Check if we're already at the goal
        double distance_to_goal = goal_global.norm();
        if (distance_to_goal < 0.1)
        {
            RCLCPP_INFO(this->get_logger(), "Reached goal position!");
            return;
        }

        // Create 2D rotation matrix for robot's orientation
        double theta = -pose_(2);
        Eigen::Matrix2f rot;
        rot << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);

        // Rotate goal position to robot's frame
        Eigen::Vector2f goal_local = rot * goal_global;

        // Calculate heading in robot's frame
        double relative_heading = normalize_angle(std::atan2(goal_local(1), goal_local(0)));
        double heading_deg = relative_heading * 180.0 / M_PI;

        RCLCPP_INFO(this->get_logger(), "Goal heading: %.2f° (local frame)", heading_deg);

        // Check if there's an obstacle in front of the robot in the direction of the goal
        bool path_blocked = false;
        double min_range = std::numeric_limits<double>::infinity();

        // Search for obstacles near the goal heading
        const double search_angle = 20.0 * M_PI / 180.0; // Search ±20 degrees around goal heading

        for (size_t i = 0; i < scan_angles.size(); ++i)
        {
            double angle_diff = std::abs(normalize_angle(scan_angles[i] - relative_heading));
            if (angle_diff < search_angle)
            {
                if (std::isfinite(laserscan_vector[i]))
                {
                    min_range = std::min(min_range, laserscan_vector[i]);
                    if (laserscan_vector[i] < 1.0)
                    { // Obstacle detection threshold
                        path_blocked = true;
                    }
                }
            }
        }

        if (!path_blocked)
        {
            // No obstacle in the way - go straight to goal
            RCLCPP_INFO(this->get_logger(), "Path to goal is clear (min range: %.2f m), proceeding directly", min_range);
            travel_to_goal();
        }
        else
        {
            // Obstacle detected - follow boundary
            RCLCPP_INFO(this->get_logger(), "Obstacle detected (range: %.2f m), following boundary", min_range);
            surround_obstacle();
        }

        pid_ref_pub_->publish(pid_ref_msg);
    }

    void travel_to_goal()
    {
        // Set waypoint directly to goal position
        pid_ref_msg.pose.position.x = goal_pose_(0);
        pid_ref_msg.pose.position.y = goal_pose_(1);

        // Set orientation towards goal
        quat.setRPY(0, 0, goal_pose_(2));
        tf2::convert(quat, pid_ref_msg.pose.orientation);
    }

    int get_scan_index_for_heading(double relative_heading, double angle_min, double angle_increment, size_t scan_size)
    {
        // Normalize the relative heading to be in the same range as laser scan angles
        relative_heading = normalize_angle(relative_heading);

        // Calculate the index
        int index = static_cast<int>((relative_heading - angle_min) / angle_increment);

        // Ensure index is within bounds
        if (index < 0)
            index = 0;
        if (index >= static_cast<int>(scan_size))
            index = scan_size - 1;

        return index;
    }

    double calculate_heading(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;

        return std::atan2(dy, dx);
    }

    Eigen::Vector3f get_point(int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(laserscan_vector.size()))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid index in get_point: %d", idx);
            return Eigen::Vector3f::Zero();
        }

        double r = laserscan_vector[idx];
        double theta = scan_angles[idx]; // Use stored angle

        Eigen::Vector3f point;
        point << r * std::cos(theta),
            r * std::sin(theta),
            0;

        return point;
    }

    std::pair<int, int> get_next_valid_pair()
    {
        if (laserscan_vector.empty())
        {
            return std::make_pair(-1, -1);
        }

        // Find closest to center point
        int center_idx = laserscan_vector.size() / 2;

        const double max_point_distance = 3.0;
        const double min_point_distance = 0.2;
        const double min_pair_separation = 0.1;

        // Search outward from center
        for (int offset = 0; offset < static_cast<int>(laserscan_vector.size() / 2); ++offset)
        {
            // Try both sides of center
            int right_idx = std::min(center_idx + offset, static_cast<int>(laserscan_vector.size()) - 1);
            int right_next = right_idx + 1;

            if (right_next < static_cast<int>(laserscan_vector.size()))
            {
                if (is_valid_pair(right_idx, right_next, min_point_distance, max_point_distance, min_pair_separation))
                {
                    return std::make_pair(right_idx, right_next);
                }
            }

            int left_idx = std::max(center_idx - offset, 0);
            int left_prev = left_idx - 1;

            if (left_prev >= 0)
            {
                if (is_valid_pair(left_prev, left_idx, min_point_distance, max_point_distance, min_pair_separation))
                {
                    return std::make_pair(left_prev, left_idx);
                }
            }
        }

        return std::make_pair(-1, -1);
    }

    bool is_valid_pair(int idx1, int idx2, double min_dist, double max_dist, double min_separation)
    {
        if (!std::isfinite(laserscan_vector[idx1]) || !std::isfinite(laserscan_vector[idx2]))
        {
            return false;
        }

        if (laserscan_vector[idx1] > max_dist || laserscan_vector[idx2] > max_dist ||
            laserscan_vector[idx1] < min_dist || laserscan_vector[idx2] < min_dist)
        {
            return false;
        }

        Eigen::Vector3f p1 = get_point(idx1);
        Eigen::Vector3f p2 = get_point(idx2);

        double separation = (p2.head<2>() - p1.head<2>()).norm();
        if (separation < min_separation)
        {
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Valid pair found: p1(%.3f, %.3f), p2(%.3f, %.3f)",
                    p1(0), p1(1), p2(0), p2(1));
        return true;
    }

    void surround_obstacle()
    {
        auto maybe_pair = get_next_valid_pair();
        if (maybe_pair.first == -1 || maybe_pair.second == -1)
        {
            RCLCPP_WARN(this->get_logger(), "No valid LaserScan pair found.");
            return;
        }

        auto [idx1, idx2] = maybe_pair;
        Eigen::Vector3f p1 = get_point(idx1);
        Eigen::Vector3f p2 = get_point(idx2);

        // Calculate tangent and normal
        Eigen::Vector2f tangent = (p2.head<2>() - p1.head<2>()).normalized();
        Eigen::Vector2f normal(-tangent(1), tangent(0)); // Left-hand normal
        double heading_angle = std::atan2(tangent(1), tangent(0));

        // Calculate midpoint and offset
        Eigen::Vector2f midpoint = (p1.head<2>() + p2.head<2>()) / 2.0;
        double obstacle_distance = (laserscan_vector[idx1] + laserscan_vector[idx2]) / 2.0;

        // Dynamic offset based on distance
        double base_offset = 0.3;
        double scale_factor = 0.2;
        double offset = std::min(base_offset + scale_factor * obstacle_distance, 0.8);

        // First calculate the offset perpendicular to the wall
        Eigen::Vector2f offset_point = midpoint + offset * normal;

        // Then add the forward offset along the wall for corner peeking
        double forward_offset = 0.2;                               // Length of the "peek" forward
        Eigen::Vector2f forward_vector = forward_offset * tangent; // Use tangent for forward motion
        Eigen::Vector2f final_point = offset_point + forward_vector;

        // Calculate waypoint in local frame
        Eigen::Vector3f local_waypoint;
        local_waypoint << final_point(0),
            final_point(1),
            heading_angle;

        // Transform to global frame
        Eigen::Vector2f local_pos = local_waypoint.head<2>();
        Eigen::Vector2f rotated_pos = rotM_.block<2, 2>(0, 0) * local_pos;
        Eigen::Vector3f waypoint;
        waypoint.head<2>() = pose_.head<2>() + rotated_pos;
        waypoint(2) = normalize_angle(pose_(2) + local_waypoint(2));

        // Set message
        set_pid_ref(waypoint);
        RCLCPP_INFO(this->get_logger(), "Published waypoint: [%.3f, %.3f, %.1f°]",
                    waypoint(0), waypoint(1), waypoint(2) * 180.0 / M_PI);
    }

    void set_pid_ref(Eigen::Vector3f v)
    {
        pid_ref_msg.pose.position.x = v(0);
        pid_ref_msg.pose.position.y = v(1);
        quat.setRPY(0, 0, v(2));
        tf2::convert(quat, pid_ref_msg.pose.orientation);
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveNavigationNode>());
    rclcpp::shutdown();
    return 0;
}