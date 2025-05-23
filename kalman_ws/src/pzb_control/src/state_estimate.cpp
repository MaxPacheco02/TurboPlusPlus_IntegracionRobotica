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

class StateEstimateNode : public rclcpp::Node
{
public:
    StateEstimateNode() : Node("state_estimate")
    {
        using namespace std::placeholders;

        this->declare_parameter("wheel_relation2", 0.0); // default wheel_relation2
        wheel_relation2 = this->get_parameter("wheel_relation2").as_double();
        this->declare_parameter("angle_ponder", 0.0); // default angle_ponder
        angle_ponder = this->get_parameter("angle_ponder").as_double();

        kalman_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pzb/kal_odom", 
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            std::bind(&StateEstimateNode::kalman_pose_update, this, _1)
        );

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const geometry_msgs::msg::Twist &msg){
                update_xid_params_cmdvel(msg.linear.x, msg.angular.z);
            });

        l_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/VelocityEncL", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const std_msgs::msg::Float32 &msg){
                if(q1.size() <= q_length)
                    q1.push(msg.data / wheel_relation2);
                if(q1.size() > q_length)
                    q1.pop();
                w1 = average(q1);
                w1_msg.data = w1;
                update_xid_params_ws(w1, w2);
            });
            
        r_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/VelocityEncR", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const std_msgs::msg::Float32 &msg){
                if(q2.size() <= q_length)
                    q2.push(msg.data);
                if(q2.size() > q_length)
                    q2.pop();
                w2 = average(q2);
                w2_msg.data = w2;
                update_xid_params_ws(w1, w2);
            });

        w1_smooth_pub_ = this->create_publisher<std_msgs::msg::Float32>("/w1_smooth", 10);
        w2_smooth_pub_ = this->create_publisher<std_msgs::msg::Float32>("/w2_smooth", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/pzb/pose", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/pzb/vel", 10);
        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/pzb/odom", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        updateTimer =
            this->create_wall_timer(10ms, std::bind(&StateEstimateNode::update, this));

        xi << 0, 0, 0;
        xi_dot << 0, 0, 0;
        
        //------------------------------------------------
        Jacobian_H << 1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0;  
        Covariance << 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0;
        Q_matrix << 0.5, 0.01, 0.01,
                    0.01, 0.5, 0.01,
                    0.01, 0.01, 0.2;
        //------------------------------------------------

        pose_stamped_msg.header.frame_id = "odom";
        odometry_msg.header.frame_id = "odom";
        joint_state_msg.header.frame_id = "odom";

        joint_state_msg.name.push_back("left_wheel_joint");
        joint_state_msg.name.push_back("right_wheel_joint");
        joint_state_msg.name.push_back("lidar_joint");

        joint_state_msg.position.push_back(0.0);
        joint_state_msg.position.push_back(0.0);
        joint_state_msg.position.push_back(0.0);
    }

private:

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr l_enc_sub_, r_enc_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr kalman_odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pose_pub_, vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr w1_smooth_pub_, w2_smooth_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    geometry_msgs::msg::Vector3 pose_msg, vel_msg;
    geometry_msgs::msg::PoseStamped pose_stamped_msg;

    std_msgs::msg::Float32 w1_msg, w2_msg;

    nav_msgs::msg::Odometry odometry_msg;

    sensor_msgs::msg::JointState joint_state_msg;

    tf2::Quaternion quat;

    double w1{0.0}, w2{0.0};
    double twist_x{0.0}, twist_z{0.0};
    double theta_l{0.0}, theta_r{0.0}, theta_lidar{0.0};

    const double r{0.05}, l{0.08}, dt{0.01};

    Eigen::Vector3f xi, xi_dot;

    double wheel_relation2{0.0};
    double angle_ponder{0.0};

    std::queue<double> q1, q2;
    int q_length{10};

    //------------------------------------------------
    Eigen::Matrix3f Covariance;
    Eigen::Matrix3f Jacobian_H;
    Eigen::Matrix3f Q_matrix; 
    //------------------------------------------------

    void kalman_pose_update(){
        a = 0;
    }

    void update() {      
        
        xi_dot << 
            twist_x* cos(xi(2)),
            twist_x* sin(xi(2)),
            twist_z;
        
        xi_dot(2) /= angle_ponder;

        // Integrate
        xi << xi + xi_dot * dt;

        // Make theta [-pi, pi]
        xi(2) = std::fmod(xi(2) + M_PI, 2*M_PI) - M_PI;

        // Set and publish ROS msgs
        pose_msg.x = xi(0);
        pose_msg.y = xi(1);
        pose_msg.z = xi(2);

        vel_msg.x = xi_dot(0);
        vel_msg.y = xi_dot(1);
        vel_msg.z = xi_dot(2);
        
        quat.setRPY(0, 0, xi(2));
        odometry_msg.pose.pose.position.x = xi(0);
        odometry_msg.pose.pose.position.y = xi(1);
        tf2::convert(quat, odometry_msg.pose.pose.orientation);

        odometry_msg.twist.twist.linear.x = twist_x;
        odometry_msg.twist.twist.angular.z = twist_z;

        //------------------------------------------------------------------------------
        Jacobian_H(0,2) = -(dt * xi_dot(1));
        Jacobian_H(1,2) = dt * xi_dot(0);
        
        Covariance = (Jacobian_H * Covariance * Jacobian_H.transpose()) + Q_matrix;

        odometry_msg.pose.covariance[0]  = Covariance(0, 0);  
        odometry_msg.pose.covariance[1]  = Covariance(0, 1);  
        odometry_msg.pose.covariance[5]  = Covariance(0, 2);  
        odometry_msg.pose.covariance[6]  = Covariance(1, 0);  
        odometry_msg.pose.covariance[7]  = Covariance(1, 1);  
        odometry_msg.pose.covariance[11] = Covariance(1, 2);  
        odometry_msg.pose.covariance[30] = Covariance(2, 0);  
        odometry_msg.pose.covariance[31] = Covariance(2, 1);  
        odometry_msg.pose.covariance[35] = Covariance(2, 2);  
        //------------------------------------------------------------------------------
        
        theta_l -= w1*dt;
        theta_r += w2*dt;

        theta_lidar += 10.0*dt;

        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.position[1] = theta_l;
        joint_state_msg.position[0] = theta_r;
        joint_state_msg.position[2] = theta_lidar;

        pose_pub_->publish(pose_msg);
        vel_pub_->publish(vel_msg);
        odometry_pub_->publish(odometry_msg);
        w1_smooth_pub_->publish(w1_msg);
        w2_smooth_pub_->publish(w2_msg);
        joint_state_pub_->publish(joint_state_msg);
    }

    double average(std::queue<double> q){
        int siz = q.size();
        double sum{0};
        while(q.size() > 0){
            sum += q.front();
            q.pop();
        }
        if(siz > 0)
            return sum / siz;
        else
            return 0;
    }

    // Obtain velocity vector params with Twist msg
    void update_xid_params_cmdvel(double lin_x, double ang_z){
        twist_x = lin_x;
        twist_z = ang_z;

        w1 = (lin_x - ang_z*l)/r;
        w2 = (lin_x + ang_z*l)/r;
    }

    // Obtain velocity vector params with wheels' ang. vel
    void update_xid_params_ws(double w_l, double w_r){
        twist_x = (r/2) * (w_l + w_r);
        twist_z = (r/(2*l)) * (w_r - w_l);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimateNode>());
    rclcpp::shutdown();
    return 0;
}
