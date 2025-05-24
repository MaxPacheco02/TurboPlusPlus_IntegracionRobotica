#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "pzb_msgs/msg/object_detected.hpp"
#include "pzb_msgs/msg/object_detected_vector.hpp"

using namespace std::chrono_literals;

class MasterNode : public rclcpp::Node
{
public:
    MasterNode() : Node("master_node")
    {
        using namespace std::placeholders;

        obj_vector_sub_ = this->create_subscription<pzb_msgs::msg::ObjectDetectedVector>(
            "/object_detected_vector", 10,
            [this](const pzb_msgs::msg::ObjectDetectedVector &msg){
                color = color_to_int[msg.object_detected_vector[0].color];
                switch(color){
                    case 0:
                        RCLCPP_ERROR(get_logger(), "STOP");
                        multiplier = 0;
                        break;
                    case 1:
                        RCLCPP_ERROR(get_logger(), "GO");
                        multiplier = 1;
                        break;
                    case 2:
                        RCLCPP_ERROR(get_logger(), "SLOW DOWN");
                        multiplier = 0.5;
                        break;
                }
        });

        w1_des_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/w1_d", 10,
            [this](const std_msgs::msg::Float32 &msg){this->w1_msg.data = msg.data * multiplier;});
        w2_des_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/w2_d", 10,
            [this](const std_msgs::msg::Float32 &msg){this->w2_msg.data = msg.data * multiplier;});

        w1_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>("/VelocitySetL", 10);
        w2_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>("/VelocitySetR", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/floor_marker", 10);

        updateTimer = this->create_wall_timer(50ms, std::bind(&MasterNode::update, this));

        marker.header.frame_id = "world";
        marker.action = 0;
        marker.id = 0;
        marker.type = marker.SPHERE;
        marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().
                        x(0.2).y(0.2).z(0.2); 
        marker.pose.position.x = -1;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
    }

private:

    rclcpp::Subscription<pzb_msgs::msg::ObjectDetectedVector>::SharedPtr obj_vector_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr w1_des_sub_, w2_des_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr w1_cmd_pub_, w2_cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    std_msgs::msg::Float32 w1_msg, w2_msg;
    visualization_msgs::msg::Marker marker;

    int color{3};
    double multiplier{1.0};

    std::map<std::string, int> color_to_int = {
        {"red", 0},
        {"green", 1},
        {"yellow", 2},
        {"", 3},
    };

    double color_list[4][4]{
        {1,0,0,1},
        {0,1,0,1},
        {1,1,0,1},
        {0.1,0.1,0.1,1},
    };

    void update() {
        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().
                        r(color_list[color][0]).
                        g(color_list[color][1]).
                        b(color_list[color][2]).
                        a(color_list[color][3]);
        marker.id = marker.id + 1;
        marker_pub_->publish(marker);
        w1_cmd_pub_->publish(this->w1_msg);
        w2_cmd_pub_->publish(this->w2_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}
