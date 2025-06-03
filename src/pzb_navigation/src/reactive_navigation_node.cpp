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

struct MarkerProps
{
    int type;
    double x, y, z, z_trans;
};

class ReactiveNavigationNode : public rclcpp::Node
{
public:
    enum class NavState
    {
        MOVE_TO_GOAL,
        SURROUND
    };

    ReactiveNavigationNode() : Node("reactive_navigation_node")
    {
        this->declare_parameter("bug_algorithm", 0);
        this->declare_parameter("wall_distance_thresh", 0.35);

        // Initialize algorithm type based on parameter
        bug_algorithm = this->get_parameter("bug_algorithm").as_int();
        wall_dist_thresh = this->get_parameter("wall_distance_thresh").as_double();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", 10,
            [this](const sensor_msgs::msg::LaserScan &msg)
            {
                angle_min = msg.angle_min;
                angle_max = msg.angle_max;

                double limit_range_max = 1.0;

                // Reduce skip_rays to get better coverage
                // skip_rays = static_cast<int>((scan_filter_angle * M_PI / 180.0) / msg.angle_increment);
                skip_rays = 1.0;
                angle_increment = msg.angle_increment * skip_rays;

                laserscan_vector.clear();
                scan_angles.clear();

                nearest_index = -1;

                for (size_t i = 0; i < msg.ranges.size(); i += skip_rays)
                {
                    double angle = msg.angle_min + i * msg.angle_increment;
                    angle = normalize_angle(angle);

                    if (std::isfinite(msg.ranges[i])
                        // &&
                        // msg.ranges[i] >= msg.range_min &&
                        // msg.ranges[i] <= limit_range_max
                        // &&  std::fabs(angle) < M_PI_2
                    )
                    {
                        laserscan_vector.push_back(msg.ranges[i]);
                        scan_angles.push_back(angle);

                        if (nearest_index == -1 || msg.ranges[i] < laserscan_vector[nearest_index])
                        {
                            nearest_index = laserscan_vector.size() - 1;
                        }
                    }
                }

                // RCLCPP_INFO(this->get_logger(), "Processed %zu valid points", laserscan_vector.size());
                // for (size_t i = 0; i < std::min(size_t(5), laserscan_vector.size()); ++i)
                // {
                //     RCLCPP_INFO(this->get_logger(), "Point %zu: range=%.3f, angle=%.1f°",
                //                 i, laserscan_vector[i], scan_angles[i] * 180.0 / M_PI);
                // }
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

                if (!received_goal)
                {
                    set_pid_ref(pose_);
                    pid_ref_pub_->publish(pid_ref_msg);
                }
            });

        nav_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/avoidance/goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped &msg)
            {
                received_goal = true;

                goal_pose_ << msg.pose.position.x,
                    msg.pose.position.y,
                    tf2::getYaw(msg.pose.orientation);

                goal_req_pose_ = pose_;
                bug_state = NavState::MOVE_TO_GOAL;
            });

        pid_ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_array", 10);

        update_timer_ = this->create_wall_timer(50ms, std::bind(&ReactiveNavigationNode::update, this));
        pid_ref_msg.header.frame_id = "odom";
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pid_ref_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    visualization_msgs::msg::MarkerArray marker_arr;
    rclcpp::TimerBase::SharedPtr update_timer_;
    tf2::Quaternion quat;
    geometry_msgs::msg::PoseStamped pid_ref_msg;

    double scan_filter_angle{5.0};
    std::vector<double> laserscan_vector;
    std::vector<double> scan_angles; // Store actual angles for each point
    double angle_min, angle_max, angle_increment;
    int nearest_index{-1};

    Eigen::Vector3f pose_, goal_pose_, goal_req_pose_;
    Eigen::Matrix3f rotM_;

    double received_goal{false};
    NavState bug_state;
    int bug_algorithm{2};
    double wall_dist_thresh{0.2};
    int skip_rays{1};

    int color_list[6][4]{
        {1, 0, 0, 1},
        {0, 1, 0, 1},
        {0, 0, 1, 1},
        {1, 1, 0, 1},
        {0, 0, 0, 1},
        {0, 0, 0, 0}};

    std::map<std::string, MarkerProps> marker_type = {
        {"round", MarkerProps{2, 0.5, 0.5, 0.5, 0}},
        {"boat", MarkerProps{2, 1.0, 1.0, 1.0, 0}},
        {"marker", MarkerProps{3, 0.03, 0.03, 0.03, 0.25}},
        {"picture", MarkerProps{1, 0.5, 0.5, 0.5, 0.25}},
    };

    void add_marker(Eigen::Vector2f p, int color)
    {
        visualization_msgs::msg::Marker marker;

        std::string type = "marker";

        int r{color_list[color][0]},
            g{color_list[color][1]},
            b{color_list[color][2]},
            a{color_list[color][3]};

        marker.header.frame_id = "base_axis";
        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(r).g(g).b(b).a(a);
        marker.action = 0;
        marker.id = marker_arr.markers.size();
        marker.type = marker_type[type].type;
        marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(marker_type[type].x).y(marker_type[type].y).z(marker_type[type].z);
        marker.pose.position.x = p(0);
        marker.pose.position.y = p(1);
        marker.pose.position.z = marker_type[type].z_trans;
        marker_arr.markers.push_back(marker);
    }

    void add_marker(Eigen::Vector3f p, int color)
    {
        Eigen::Vector2f pp{p(0), p(1)};
        add_marker(pp, color);
    }

    void update()
    {
        if (!received_goal)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal not received");
            return;
        }
        if (nearest_index >= static_cast<int>(laserscan_vector.size()) || laserscan_vector.size() == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Nearest scan index unsuitable: %d >= %d", nearest_index, laserscan_vector.size());
            return;
        }

        marker_arr.markers.clear();

        double global_heading = calculate_heading(pose_, goal_pose_);
        double local_heading = normalize_angle(global_heading - pose_(2));

        bool path_clear = is_goal_clear();
        RCLCPP_INFO(this->get_logger(), "Path clear? %d", path_clear);
        RCLCPP_INFO(this->get_logger(), "Cond 2: %d", laserscan_vector[nearest_index] < wall_dist_thresh);
        RCLCPP_INFO(this->get_logger(), "Cond 3: %d", std::fabs(ang_distance(scan_angles[nearest_index], local_heading)) < 2 * M_PI / 8);

        bug_algorithm = this->get_parameter("bug_algorithm").as_int();

        // Discuss machine's state
        if ((!path_clear) || ((laserscan_vector[nearest_index] < wall_dist_thresh) && (std::fabs(ang_distance(scan_angles[nearest_index], local_heading)) < 3 * M_PI / 8)))
            bug_state = NavState::SURROUND;
        else
        {
            if (bug_algorithm == 0)
            {
                // Bug 0 just needs a clear path to nav to goal.
                bug_state = NavState::MOVE_TO_GOAL;
            }
            else
            {
                // Bug 2 also needs the robot to be along the line created when the last goal was received.
                if (calculate_crosstrack_error(pose_, goal_req_pose_, goal_pose_) < 0.1)
                    bug_state = NavState::MOVE_TO_GOAL;
            }
        }

        // Act according to state
        switch (bug_state)
        {
        case NavState::MOVE_TO_GOAL:
            set_pid_ref(goal_pose_);
            break;
        default:
            surround();
            break;
        }

        // TODO: Remove

        add_marker(get_point(0), 4);
        add_marker(get_point(laserscan_vector.size() - 1), 4);
        add_marker(get_point(-angle_min / angle_increment), 3);

        marker_pub_->publish(marker_arr);
        pid_ref_pub_->publish(pid_ref_msg);
    }

    int get_min(std::vector<double> v)
    {
        if (v[0] < v[1])
            return 0;
        return 1;
    }

    double calculate_crosstrack_error(const Eigen::Vector3f &pose,
                                      const Eigen::Vector3f &point1,
                                      const Eigen::Vector3f &point2)
    {
        // Extract 2D positions (ignore z/theta component)
        Eigen::Vector2f p(pose[0], pose[1]);
        Eigen::Vector2f a(point1[0], point1[1]);
        Eigen::Vector2f b(point2[0], point2[1]);

        // Vector from start point to end point
        Eigen::Vector2f ab = b - a;

        // Vector from start point to pose
        Eigen::Vector2f ap = p - a;

        // Check if line segment has zero length
        double ab_length = ab.norm();
        if (ab_length < 1e-6)
        {
            // Return distance to either point if segment is degenerate
            return ap.norm();
        }

        // Calculate normalized projection
        double t = ap.dot(ab) / (ab_length * ab_length);

        // If projection is before start point
        if (t < 0.0)
        {
            return ap.norm();
        }
        // If projection is after end point
        else if (t > 1.0)
        {
            return (p - b).norm();
        }

        // Project point onto line segment
        Eigen::Vector2f projection = a + t * ab;

        // Return perpendicular distance
        return (p - projection).norm();
    }

    // Check if the lidar's global scan distance towards the goal is greater than a minimum range or the distance from the pose.
    bool is_goal_clear()
    {
        double global_heading = calculate_heading(pose_, goal_pose_);
        double local_heading = normalize_angle(global_heading - pose_(2));
        double goal_dist = dist(pose_, goal_pose_);

        double min_ang_distance = M_PI;
        int closest_idx = -1;
        for (int i = 0; i < scan_angles.size(); i++)
        {
            double ang = ang_distance(local_heading, scan_angles[i]);
            if (std::fabs(ang) < min_ang_distance)
            {
                min_ang_distance = std::fabs(ang);
                closest_idx = i;
            }
        }

        // ang = min + inc * i
        int idx = (local_heading - angle_min)  /angle_increment;

        // RCLCPP_INFO(this->get_logger(), "Local heading towards goal: %f", local_heading);
        // RCLCPP_INFO(this->get_logger(), "Nearest registered heading: %f", scan_angles[closest_idx]);
        // RCLCPP_INFO(this->get_logger(), "Closest ang's error: %f", min_ang_distance);

        // if (std::fabs(min_ang_distance) > M_PI)
        //     return true; // The nearest obstacle is on a different direction than the goal

        std::cout << "goal dist: " << goal_dist << std::endl;
        Eigen::Vector2f goals_local_v = get_point(idx);
        std::cout << "goal local head" << goals_local_v(0) << "," << goals_local_v(1) << std::endl;
        std::cout << "inf goal?: " << (laserscan_vector[idx] > 20.0) << std::endl;
        add_marker(goals_local_v, 3);

        // This is too big. Simulating infinite
        if((laserscan_vector[idx] > 20.0) || goals_local_v.norm() < 0.01)
            return true;
        if (laserscan_vector[idx] > goal_dist ||
            laserscan_vector[idx] > 1.0)
            return true; // There is an obstacle on the goal's direction but its further than the goal
        return false;    // The is an obstacle between the pzb and the goal
    }

    double dist(Eigen::Vector3f p1, Eigen::Vector3f p2)
    {
        return std::sqrt(
            std::pow(p1(0) - p2(0), 2) +
            std::pow(p1(1) - p2(1), 2));
    }

    double dist(Eigen::Vector2f p1, Eigen::Vector2f p2)
    {
        return std::sqrt(
            std::pow(p1(0) - p2(0), 2) +
            std::pow(p1(1) - p2(1), 2));
    }

    double ang_distance(double ang1, double ang2)
    {
        return normalize_angle(ang1 - ang2);
    }

    Eigen::Vector3f get_offset_point(int base, int sense)
    {
        int reversed = 1;
        int sense_addition = sense;
        Eigen::Vector2f p1 = get_point(base);

        int next_index = (base + sense_addition + laserscan_vector.size()) % laserscan_vector.size();
        Eigen::Vector2f p2 = get_point(next_index);

        // In case we are near a corner, the next idx might lead to a very far away point.
        if (dist(p1, p2) > 0.1)
        {
            // reversed = -1;
            next_index = (base - sense_addition + laserscan_vector.size()) % laserscan_vector.size();
            p2 = p1;
            p1 = get_point(next_index);
        }

        // Calculate tangent and normal
        Eigen::Vector2f tangent = (p2 - p1).normalized();
        Eigen::Vector2f normal(-tangent(1), tangent(0)); // Left-hand normal
        double heading_angle = std::atan2(tangent(1), tangent(0));

        // Calculate midpoint and offset
        Eigen::Vector2f midpoint = (p1 + p2) / 2.0;
        double obstacle_distance = (laserscan_vector[base] + laserscan_vector[next_index]) / 2.0;

        // Dynamic offset based on distance
        double base_offset = 0.25;
        double scale_factor = 0.2;
        double offset = std::min(base_offset + scale_factor * obstacle_distance, 0.3);

        // Calculate the offset perpendicular to the wall
        Eigen::Vector2f offset_point0 = p1 + offset * normal * sense * reversed;

        double forward_offset = 0.1;                               // Length of the "peek" forward
        Eigen::Vector2f forward_vector = forward_offset * tangent; // Use tangent for forward motion

        Eigen::Vector2f offset_point = offset_point0 + forward_vector;

        add_marker(p1, 0);
        add_marker(p2, 1);
        add_marker(offset_point0, 2);
        add_marker(offset_point, 3);

        return Eigen::Vector3f{offset_point(0), offset_point(1), 0};
    }

    void surround()
    {
        int senses[2]{-1, 1};
        Eigen::Vector3f offset_point1 = get_offset_point(nearest_index, senses[0]);
        Eigen::Vector3f offset_point2 = get_offset_point(nearest_index, senses[1]);

        // Choose correct sense based on which one is closer to the goal ...
        // std::vector<double> points_v{
        //     dist(pose_ + rotM_ * offset_point1, goal_pose_),
        //     dist(pose_ + rotM_ * offset_point2, goal_pose_)};

        // ... or choose correct sense based on which one requires the least yaw
        std::vector<double> points_v{
            std::fabs(normalize_angle(std::atan2(offset_point1(1), offset_point1(0)))),
            std::fabs(normalize_angle(std::atan2(offset_point2(1), offset_point2(0))))};

        // RCLCPP_ERROR(this->get_logger(), "Heading options: %f, %f", points_v[0], points_v[1]);


        std::vector<Eigen::Vector3f> point_list{offset_point1, offset_point2};

        int correct_sense = get_min(points_v);

        Eigen::Vector3f local_wp = point_list[correct_sense];

        Eigen::Vector2f p1 = get_point(nearest_index);
        int next_idx = (nearest_index + senses[correct_sense] + laserscan_vector.size()) % laserscan_vector.size();
        Eigen::Vector2f p2 = get_point(next_idx);

        if (dist(p1, p2) > 0.1)
        {
            next_idx = (nearest_index - senses[correct_sense] + laserscan_vector.size()) % laserscan_vector.size();
            p2 = p1;
            p1 = get_point(next_idx);
        }

        local_wp(2) = calculate_heading(p1, p2);

        Eigen::Vector3f wp = pose_ + rotM_ * local_wp;

        set_pid_ref(wp);

        // add_marker(p1, 0);
        // add_marker(p2, 1);
        // add_marker(point_list[0], 3);
        // add_marker(point_list[1], 3);
        // add_marker(final_point, 3);
    }

    double calculate_heading(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;

        return std::atan2(dy, dx);
    }

    double calculate_heading(Eigen::Vector2f p1, Eigen::Vector2f p2)
    {
        Eigen::Vector2f diff = p2 - p1;
        return std::atan2(diff(1), diff(0));
    }

    double calculate_heading(Eigen::Vector3f p1, Eigen::Vector3f p2)
    {
        Eigen::Vector3f diff = p2 - p1;
        return std::atan2(diff(1), diff(0));
    }

    Eigen::Vector2f get_point(int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(laserscan_vector.size()))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid index in get_point: %d", idx);
            return Eigen::Vector2f::Zero();
        }

        double r = laserscan_vector[idx];
        double theta = scan_angles[idx];

        Eigen::Vector2f point;
        point << r * std::cos(theta),
            r * std::sin(theta);

        return point;
    }

    Eigen::Vector3f get_point3(int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(laserscan_vector.size()))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid index in get_point: %d", idx);
            return Eigen::Vector3f::Zero();
        }

        double r = laserscan_vector[idx];
        double theta = scan_angles[idx];

        Eigen::Vector3f point;
        point << r * std::cos(theta),
            r * std::sin(theta), 0;

        return point;
    }

    Eigen::Vector2f midpoint(Eigen::Vector2f p1, Eigen::Vector2f p2)
    {
        Eigen::Vector2f p = (p1 + p2) / 2;
        return p;
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    void set_pid_ref(Eigen::Vector3f v)
    {
        pid_ref_msg.header.stamp = this->get_clock()->now();

        pid_ref_msg.pose.position.x = v(0);
        pid_ref_msg.pose.position.y = v(1);
        quat.setRPY(0, 0, v(2));
        tf2::convert(quat, pid_ref_msg.pose.orientation);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveNavigationNode>());
    rclcpp::shutdown();
    return 0;
}