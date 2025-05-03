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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using namespace std::chrono_literals;

class DeadReckoingNode : public rclcpp::Node
{
public:
    DeadReckoingNode() : Node("dead_reckoning")
    {
        using namespace std::placeholders;
        ns_ = this->get_namespace();

        l_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "w_l", 10,
            [this](const std_msgs::msg::Float32 &msg){
                w1 = msg.data;
                update_xid_params_ws();
            });

        r_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "w_r", 10,
            [this](const std_msgs::msg::Float32 &msg){
                w2 = msg.data;
                update_xid_params_ws();
            });

        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        update_timer_ =
            this->create_wall_timer(10ms, std::bind(&DeadReckoingNode::update, this));

        xi << 0, 0, 0;
        xi_dot << 0, 0, 0;

        odometry_msg.header.frame_id = "odom";
    }

private:

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr l_enc_sub_, r_enc_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    rclcpp::TimerBase::SharedPtr update_timer_;

    nav_msgs::msg::Odometry odometry_msg;

    tf2::Quaternion quat;

    std::string ns_;

    double w1{0.0}, w2{0.0};
    double twist_x{0.0}, twist_z{0.0};

    const double r{0.05}, l{0.08}, dt{0.01};

    Eigen::Vector3f xi, xi_dot;

    void update() {      
        // RCLCPP_ERROR(get_logger(), "%f,%f,%f - %f, %f", xi(0),xi(1),xi(2), twist_x, twist_z);

        xi_dot << 
            twist_x* cos(xi(2)),
            twist_x* sin(xi(2)),
            twist_z;
        
        // Integrate
        xi << xi + xi_dot * dt;

        // Make theta [-pi, pi]
        xi(2) = std::fmod(xi(2) + M_PI, 2*M_PI) - M_PI;

        // Set and publish ROS msgs
        quat.setRPY(0, 0, xi(2));
        odometry_msg.pose.pose.position.x = xi(0);
        odometry_msg.pose.pose.position.y = xi(1);
        tf2::convert(quat, odometry_msg.pose.pose.orientation);

        odometry_msg.twist.twist.linear.x = twist_x;
        odometry_msg.twist.twist.angular.z = twist_z;

        odometry_msg.header.stamp = this->get_clock()->now();

        odometry_pub_->publish(odometry_msg);
    }

    // Obtain velocity vector params with wheels' ang. vel
    void update_xid_params_ws(){
        twist_x = (r/2) * (w1 + w2);
        twist_z = (r/(2*l)) * (w2 - w1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeadReckoingNode>());
    rclcpp::shutdown();
    return 0;
}
