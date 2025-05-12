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

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class GzTranslator : public rclcpp::Node
{
public:
    GzTranslator() : Node("gz_translator")
    {
        using namespace std::placeholders;
        ns_ = this->get_namespace();

        // gz_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "/gz/joints", 10,
        //     [this](const sensor_msgs::msg::JointState &msg){
        //       joint_state_msg.position[0] = std::round(msg.position[1] * 1000.0) / 1000.0;
        //       joint_state_msg.position[1] = -std::round(msg.position[0] * 1000.0) / 1000.0;
        //     });

        gz_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/gz/joints", 10, std::bind(&GzTranslator::update, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/pzb/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist &msg){
                cmd_vel_msg.linear.x = msg.linear.x * gz_factor;
                cmd_vel_msg.angular.z = msg.angular.z * gz_factor;
            });

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/pzb_gz_drive", 10);
        
        // updateTimer =
        //     this->create_wall_timer(10ms, std::bind(&GzTranslator::update, this));

        joint_state_msg.header.frame_id = ns_ + "/base_link";

        joint_state_msg.name.push_back("left_wheel_joint");
        joint_state_msg.name.push_back("right_wheel_joint");
        joint_state_msg.name.push_back("lidar_joint");

        joint_state_msg.position.push_back(0.0);
        joint_state_msg.position.push_back(0.0);
        joint_state_msg.position.push_back(0.0);
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gz_joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    sensor_msgs::msg::JointState joint_state_msg;
    geometry_msgs::msg::Twist cmd_vel_msg;

    std::string ns_;

    double theta_l{0.0}, theta_r{0.0}, theta_lidar{0.0};
    double dt{0.01};
    double gz_factor{1.5};

    void update(const sensor_msgs::msg::JointState & msg) {

        joint_state_msg.position[0] = std::round(msg.position[1] * 1000.0) / 1000.0;
        joint_state_msg.position[1] = -std::round(msg.position[0] * 1000.0) / 1000.0;

        theta_lidar += 10.0*dt;

        joint_state_msg.header.stamp = msg.header.stamp;

        joint_state_msg.position[2] = theta_lidar;

        joint_state_pub_->publish(joint_state_msg);
        cmd_vel_pub_->publish(cmd_vel_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GzTranslator>());
    rclcpp::shutdown();
    return 0;
}
