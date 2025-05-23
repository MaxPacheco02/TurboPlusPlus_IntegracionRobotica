#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <queue> 
#include <iostream>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "aru_interfaces/msg/aruco_list.hpp"

// GAYMAN LOCALIZATION (EXTENDED)

using namespace std::chrono_literals;

class KalmanLocalizationNode : public rclcpp::Node {

public:
    KalmanLocalizationNode() 
    : Node("kalman_localization"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        
        using namespace std::placeholders;

        odometry_msgs_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pzb/odom", 
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            std::bind(&KalmanLocalizationNode::kalman_local, this, _1)
        );

        aru_sub_ = this->create_subscription<aru_interfaces::msg::ArucoList>(
            "pzb/aru_sys",10,
            std::bind(&KalmanLocalizationNode::aru_sys_id, this,_1)
        );

        kalman_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/pzb/kal_odom", 10);

        // ID'S 0,1,2, etc.
        aruco_map[0] = Eigen::Vector2f(1, 1);
        aruco_map[1] = Eigen::Vector2f(-0.7, 3.3);
        aruco_map[2] = Eigen::Vector2f(4.2, -1.8);

        R_ << 0.0, 0.0,
              0.0, 0.0;

        nearest_id = -2;
        z_k << 0.0, 0.0;
        
        miu_k.setZero();
        miu_hat.setZero();
        z_k.setZero();
        z_hat.setZero();
        sigma_k.setZero();
        sigma_hat.setZero();
        Z_k.setZero();
        G_k.setZero();
        K_k.setZero();  

        kal_odometry_msg.header.frame_id = "odom";
    }

private: 

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_msgs_;
    rclcpp::Subscription<aru_interfaces::msg::ArucoList>::SharedPtr aru_sub_; 
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kalman_odometry_pub_;

    nav_msgs::msg::Odometry kal_odometry_msg;
    geometry_msgs::msg::TransformStamped tf_aru;
 
    std::unordered_map<int, Eigen::Vector2f> aruco_map;

    tf2::Quaternion quat;

    Eigen::Vector3f miu_k; 
    Eigen::Vector3f miu_hat;
    Eigen::Vector2f z_k;
    Eigen::Vector2f z_hat;

    Eigen::Matrix3f sigma_k;
    Eigen::Matrix3f sigma_hat;
    Eigen::Matrix2f Z_k;
    Eigen::Matrix2f R_;
    Eigen::Matrix<float, 2, 3> G_k;
    Eigen::Matrix<float, 3, 2> K_k;

    int nearest_id;

    void kalman_local(const nav_msgs::msg::Odometry &msg) {
    
        // POSE DE PZ 
        float yaw = tf2::getYaw(msg.pose.pose.orientation);
        miu_hat << msg.pose.pose.position.x, msg.pose.pose.position.y, yaw;

        // COVARIANZA DE POSE DE PZ
        sigma_hat << msg.pose.covariance[0],  msg.pose.covariance[1],  msg.pose.covariance[5], 
                     msg.pose.covariance[6],  msg.pose.covariance[7],  msg.pose.covariance[11],
                     msg.pose.covariance[30], msg.pose.covariance[31], msg.pose.covariance[35];

        auto it = aruco_map.find(nearest_id);

        // REVISAR SI EL ID EXISTE EN EL MAPA 
        if (it != aruco_map.end()) {
            const auto& marker_pos = it->second;
            calc_ZHat(marker_pos.x(), marker_pos.y());
            calc_Gradient_G(marker_pos.x(), marker_pos.y());
            calc_Z();
            calc_Kalman_Gain();
            calc_Miu();
            calc_Sigma();
        } 

        kal_odometry_msg.pose.pose.position.x = miu_k(0);
        kal_odometry_msg.pose.pose.position.y = miu_k(1);
        quat.setRPY(0, 0, miu_k(2));
        tf2::convert(quat, kal_odometry_msg.pose.pose.orientation);
        
        kal_odometry_msg.twist.twist.linear.x = msg.twist.twist.linear.x;
        kal_odometry_msg.twist.twist.angular.z = msg.twist.twist.angular.z;

        kal_odometry_msg.pose.covariance[0]  = sigma_k(0, 0);  
        kal_odometry_msg.pose.covariance[1]  = sigma_k(0, 1);  
        kal_odometry_msg.pose.covariance[5]  = sigma_k(0, 2);  
        kal_odometry_msg.pose.covariance[6]  = sigma_k(1, 0);  
        kal_odometry_msg.pose.covariance[7]  = sigma_k(1, 1);  
        kal_odometry_msg.pose.covariance[11] = sigma_k(1, 2);  
        kal_odometry_msg.pose.covariance[30] = sigma_k(2, 0);  
        kal_odometry_msg.pose.covariance[31] = sigma_k(2, 1);  
        kal_odometry_msg.pose.covariance[35] = sigma_k(2, 2);
        
        kalman_odometry_pub_->publish(kal_odometry_msg);

        RCLCPP_INFO(this->get_logger(), "miu_hat = [%.2f, %.2f, %.2f]", miu_hat(0), miu_hat(1), miu_hat(2));
        RCLCPP_INFO(this->get_logger(), "miu_k = [%.2f, %.2f, %.2f]", miu_k(0), miu_k(1), miu_k(2));
    }

    void aru_sys_id(const aru_interfaces::msg::ArucoList &msg){
        if (tf_buffer_.canTransform("base_axis","aruco_center",tf2::TimePointZero)) {
            tf_aru = tf_buffer_.lookupTransform("base_axis","aruco_center",tf2::TimePointZero);
            if(tf_aru.transform.rotation != msg.markers[0].pose.orientation){
                nearest_id = msg.markers[1].id;
                float yaw = tf2::getYaw(tf_aru.transform.rotation);
                z_k << std::hypot(tf_aru.transform.translation.x, tf_aru.transform.translation.y), yaw;
            }else{
                z_k << 0.0, 0.0;
                nearest_id = -2;
            }
        }
    }

    void calc_ZHat(double m_x_i, double m_y_i) {
        double dx = m_x_i - miu_hat(0);
        double dy = m_y_i - miu_hat(1);

        double range = std::hypot(dx, dy);
        double bearing = std::atan2(dy, dx) - miu_hat(2);

        z_hat << range, bearing;    
    }

    void calc_Gradient_G(double m_x_i, double m_y_i) {
        double delta_x = m_x_i - miu_hat(0);
        double delta_y = m_y_i - miu_hat(1);

        double p = (delta_x * delta_x) + (delta_y * delta_y);
        double sqrt_p = std::sqrt(p);

        G_k(0, 0) = -delta_x / sqrt_p;
        G_k(0, 1) = -delta_y / sqrt_p;
        G_k(0, 2) = 0.0;

        G_k(1, 0) = delta_y / p;
        G_k(1, 1) = -delta_x / p;
        G_k(1, 2) = -1.0;
    }

    void calc_Z() {
        Z_k = G_k * sigma_hat * G_k.transpose() + R_;
    }

    void calc_Kalman_Gain() {
        K_k = sigma_hat * G_k.transpose() * Z_k.inverse();
    }

    void calc_Miu() {
        miu_k = miu_hat + K_k * (z_k - z_hat);
    }

    void calc_Sigma() {
        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        sigma_k = (I - K_k * G_k) * sigma_hat;
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
