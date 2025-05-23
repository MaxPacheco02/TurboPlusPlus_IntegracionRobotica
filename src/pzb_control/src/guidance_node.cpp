#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"

#include "pzb_msgs/msg/signal.hpp"

#include "std_srvs/srv/empty.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "PID.cpp"

using namespace std::chrono_literals;

struct Waypoint {
  double x, y;
};

class GuidanceNode : public rclcpp::Node
{
public:
    GuidanceNode() : Node("guidance_node")
    {
        using namespace std::placeholders;

        this->declare_parameter("wheel_relation", 1.0); // default wheel_relation
        wheel_relation = this->get_parameter("wheel_relation").as_double();
        this->declare_parameter("KP", 1.0); // default KP
        KP = this->get_parameter("KP").as_double();
        this->declare_parameter("KI", 1.0); // default KI
        KI = this->get_parameter("KI").as_double();
        this->declare_parameter("KD", 1.0); // default KD
        KD = this->get_parameter("KD").as_double();
        this->declare_parameter("u_max", 2.); // default u_max
        u_max = this->get_parameter("u_max").as_double();
        this->declare_parameter("u_min", 2.0); // default u_min
        u_min = this->get_parameter("u_min").as_double();

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/pzb_pose", 10,
            [this](const geometry_msgs::msg::Vector3 &msg){ this->pose = msg; });

        signal_sub_ = this->create_subscription<pzb_msgs::msg::Signal>(
            "/signal_detected", 10,
            [this](const pzb_msgs::msg::Signal &msg){ 
                switch(msg.signal){
                    case pzb_msgs::msg::Signal::NONE:
                        vel_multiplier = 1.;
                        break;
                    case pzb_msgs::msg::Signal::SLOW:
                        vel_multiplier = 0.5;
                        break;
                    case pzb_msgs::msg::Signal::RED:
                        vel_multiplier = 0.;
                        break;
                    case pzb_msgs::msg::Signal::YELLOW:
                        vel_multiplier = 0.5;
                        break;
                    case pzb_msgs::msg::Signal::GREEN:
                        vel_multiplier = 1.;
                        break;
                }
            });

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/pzb/path_to_follow", 10,
            [this](const nav_msgs::msg::Path &msg) { 
                if(!same_msg(wp_list, msg)){
                    // RCLCPP_INFO(get_logger(), "new wp!");
                    wp_i = 0;
                }
                wp_list = get_wp_list(msg); 
            });

        w1_des_pub_ = this->create_publisher<std_msgs::msg::Float32>("/VelocitySetL", 10);
        w2_des_pub_ = this->create_publisher<std_msgs::msg::Float32>("/VelocitySetR", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        start_srv = this->create_service<std_srvs::srv::Empty>("pzb_start", std::bind(&GuidanceNode::pzb_start, this, _1, _2));
        stop_srv = this->create_service<std_srvs::srv::Empty>("pzb_stop", std::bind(&GuidanceNode::pzb_stop, this, _1, _2));

        updateTimer =
            this->create_wall_timer(50ms, std::bind(&GuidanceNode::update, this));

        controller = PID(dt, KP, KI, KD, u_max, u_min);
        controller.updateReferences(0);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<pzb_msgs::msg::Signal>::SharedPtr signal_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr w1_des_pub_, w2_des_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv, stop_srv;

    rclcpp::TimerBase::SharedPtr updateTimer;

    std_msgs::msg::Float32 w1_des_msg, w2_des_msg;
    geometry_msgs::msg::PoseStamped wp_stamped_msg;
    geometry_msgs::msg::Vector3 pose;
    geometry_msgs::msg::Twist cmd_vel_msg;

    PID controller;

    float motors_enabled{1.};

    double psi_d{0.0}, vel_d{0.0}, ang_vel_d{0.0};

    const double change_wp_dist{0.1};
    const double r{0.05}, l{0.08};
    double wheel_relation;
    double last_vel_d{0.};
    double dt{0.05};
    double KP{1.0}, KI{1.0}, KD{1.0};
    double u_max{10.0}, u_min{-10.0};

    int wp_i{0};
    float vel_multiplier{1.};

    std::vector<Waypoint> wp_list;

    bool same_msg(std::vector<Waypoint> wps, nav_msgs::msg::Path msg){
        if(wps.size() != msg.poses.size())
            return false;
        for(int i = 0 ; i < wps.size() ; i++){
            if((wps[i].x != msg.poses[i].pose.position.x) ||
                (wps[i].y != msg.poses[i].pose.position.y))
                return false;
        }
        return true;
    }

    float distance(geometry_msgs::msg::Vector3 pos, Waypoint wp){
        return sqrt(pow(pos.x - wp.x, 2) + pow(pos.y - wp.y, 2));   
    }

    float get_angle_diff(double psi_1, double psi_2){
        double angle_diff = std::fmod((psi_1 - psi_2 + M_PI), 2*M_PI) - M_PI;
        return angle_diff < -M_PI ? angle_diff + 2 * M_PI : angle_diff;        
    }

    std::vector<Waypoint> get_wp_list(nav_msgs::msg::Path path) {
        std::vector<Waypoint> wp_l;
        for(int i = 0 ; i < path.poses.size() ; i++){
            Waypoint wp{path.poses[i].pose.position.x, path.poses[i].pose.position.y};
            wp_l.push_back(wp);
        }
        return wp_l;
    }

    void update() {        
        // Look for the waypoint indexes        
        while(wp_i < wp_list.size() && 
            (distance(this->pose, wp_list[wp_i]) < change_wp_dist))
            wp_i++;

        // Update the desired psi
        if(wp_list.size() != 0){
            psi_d = std::atan2((wp_list[wp_i].y - this->pose.y), 
                (wp_list[wp_i].x - this->pose.x));

            vel_d = 0.3 * vel_multiplier;
            // vel_d = 0.06;
            controller.saturateManipulation(get_angle_diff(psi_d, this->pose.z));
            ang_vel_d = -controller.u_;

            double divid = (1. + std::fabs(ang_vel_d) * 0.9);

            vel_d /= (divid*divid*divid);

            // if(std::fabs(ang_vel_d) > 0.1)
            //     vel_d/=0.7;
            // else if(std::fabs(ang_vel_d) > 0.2)
            //     vel_d/=2.;
            // else if(std::fabs(ang_vel_d) > 0.3)
            //     vel_d/=2.5;

            if(wp_i == wp_list.size()) {
                vel_d = 0.0;
                ang_vel_d = 0.0;
            }

            vel_d = last_vel_d + std::clamp(vel_d - last_vel_d, -1., 0.01);

            // RCLCPP_INFO(get_logger(), "u: %f", ang_vel_d);
            // RCLCPP_INFO(get_logger(), "wp_i: %d, size: %d, vel: %f, ang_v: %f", wp_i, wp_list.size(), vel_d, ang_vel_d);
            // RCLCPP_INFO(get_logger(), "From %f, %f to %f, %f", this->pose.x, this->pose.y, wp_list[wp_i].x, wp_list[wp_i].y);

            this->w1_des_msg.data = motors_enabled * wheel_relation * (vel_d - ang_vel_d * l) / r;
            this->w2_des_msg.data = motors_enabled * (vel_d + ang_vel_d * l) / r;

            w1_des_pub_->publish(this->w1_des_msg);
            w2_des_pub_->publish(this->w2_des_msg);

            this->cmd_vel_msg.linear.x = vel_d;
            this->cmd_vel_msg.angular.z = ang_vel_d;
            cmd_vel_pub_->publish(this->cmd_vel_msg);

            last_vel_d = vel_d;
        }
    }

    void pzb_start(const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        motors_enabled = 1.0;
    }

    void pzb_stop(const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        motors_enabled = 0.0;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuidanceNode>());
    rclcpp::shutdown();
    return 0;
}
