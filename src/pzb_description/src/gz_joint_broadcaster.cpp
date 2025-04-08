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

class GzJointBroadcaster : public rclcpp::Node
{
public:
    GzJointBroadcaster() : Node("gz_joint_broadcaster")
    {
        using namespace std::placeholders;

        gz_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/gz_joints", 10,
            [this](const sensor_msgs::msg::JointState &msg){
              joint_state_msg.position[0] = std::round(msg.position[1] * 1000.0) / 1000.0;
              joint_state_msg.position[1] = -std::round(msg.position[0] * 1000.0) / 1000.0;
            });

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        updateTimer =
            this->create_wall_timer(10ms, std::bind(&GzJointBroadcaster::update, this));

        joint_state_msg.header.frame_id = "odom";

        joint_state_msg.name.push_back("left_wheel_joint");
        joint_state_msg.name.push_back("right_wheel_joint");
        joint_state_msg.name.push_back("lidar_joint");

        joint_state_msg.position.push_back(0.0);
        joint_state_msg.position.push_back(0.0);
        joint_state_msg.position.push_back(0.0);
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gz_joint_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    sensor_msgs::msg::JointState joint_state_msg;

    double theta_l{0.0}, theta_r{0.0}, theta_lidar{0.0};
    double dt{0.01};

    void update() {      

        theta_lidar += 10.0*dt;

        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.position[2] = theta_lidar;

        joint_state_pub_->publish(joint_state_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GzJointBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
