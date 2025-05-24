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

#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "aru_interfaces/msg/aruco_list.hpp"

// GAYMAN LOCALIZATION (EXTENDED)

using namespace std::chrono_literals;

class StateEstimateNode : public rclcpp::Node
{
public:
    StateEstimateNode() : Node("state_estimate"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        using namespace std::placeholders;

        this->declare_parameter("wheel_relation2", 0.0); // default wheel_relation2
        wheel_relation2 = this->get_parameter("wheel_relation2").as_double();
        this->declare_parameter("angle_ponder", 0.0); // default angle_ponder
        angle_ponder = this->get_parameter("angle_ponder").as_double();

        aru_sub_ = this->create_subscription<aru_interfaces::msg::ArucoList>(
            "pzb/aru_sys",10,
            std::bind(&StateEstimateNode::aru_sys_id, this,_1));

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

        updateTimer = this->create_wall_timer(10ms, std::bind(&StateEstimateNode::update, this));

        miu_hat << 0, 0, 0;
        miu_hat_dot << 0, 0, 0;
        
        //------------------------------------------------
        H_k << 1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0;  
        sigma_hat << 1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 0.1;
        Q_k << 0.5, 0.01, 0.01,
               0.01, 0.5, 0.01,
               0.01, 0.01, 0.2;
        //------------------------------------------------

        // cp desde kalman_localization.cp
        //------------------------------------------------
        // ID'S 0,1,2, etc.
        aruco_map[0] = Eigen::Vector2f(1, 1);
        aruco_map[1] = Eigen::Vector2f(-0.7, 3.3);
        aruco_map[2] = Eigen::Vector2f(4.2, -1.8);

        R_k << 0.05, 0.0,
              0.0, 0.05;


        nearest_id = -2;
        z_k << 0.0, 0.0;
        
        miu_k.setZero();
        miu_hat.setZero();
        z_k.setZero();
        z_hat.setZero();
        sigma_k.setZero();
        Z_k.setZero();
        G_k.setZero();
        K_k.setZero();  
        //------------------------------------------------
        // cp desde kalman_localization

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

    rclcpp::Subscription<aru_interfaces::msg::ArucoList>::SharedPtr aru_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr l_enc_sub_, r_enc_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

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

    double wheel_relation2{0.0};
    double angle_ponder{0.0};

    std::queue<double> q1, q2;
    int q_length{10};

    //------------------------------------------------
    // Declaracion variables globales
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::msg::TransformStamped tf_aru;
    
    std::unordered_map<int, Eigen::Vector2f> aruco_map;

    Eigen::Vector3f miu_k;
    Eigen::Vector3f miu_hat;
    Eigen::Vector3f miu_hat_dot;
    Eigen::Vector2f z_k;
    Eigen::Vector2f z_hat;
    Eigen::Matrix3f sigma_k;
    Eigen::Matrix3f sigma_hat;
    Eigen::Matrix2f Z_k;
    Eigen::Matrix2f R_k;
    Eigen::Matrix3f H_k;
    Eigen::Matrix3f Q_k; 
    Eigen::Matrix<float, 2, 3> G_k;
    Eigen::Matrix<float, 3, 2> K_k;

    int nearest_id;
    //------------------------------------------------

    void update() {      
        
        miu_hat_dot << twist_x * cos(miu_k(2)),
                       twist_x * sin(miu_k(2)),
                       twist_z;
        
        miu_hat_dot(2) /= angle_ponder;

        // Integrate
        miu_hat << miu_k + miu_hat_dot * dt;

        // Make theta [-pi, pi] 
        miu_hat(2) = std::fmod(miu_hat(2) + M_PI, 2*M_PI) - M_PI;

        //------------------------------------------------------------------------------
        //kalman local start
        H_k(0,2) = -(dt * miu_hat_dot(1));
        H_k(1,2) = dt * miu_hat_dot(0);
        
        sigma_hat = (H_k * sigma_hat * H_k.transpose()) + Q_k;

        auto it = aruco_map.find(nearest_id);

        // REVISAR SI EL ID EXISTE EN EL MAPA 
        if (it != aruco_map.end()) {
            RCLCPP_INFO(this->get_logger(), "HELLO WORLD FROM INSIDE THE KALMAN CORRECTION PHASE!!");
            const auto& marker_pos = it->second;
            calc_ZHat(marker_pos.x(), marker_pos.y());
            calc_Gradient_G(marker_pos.x(), marker_pos.y());
            calc_Z();
            calc_Kalman_Gain();
            calc_Miu();
            calc_Sigma();
        } else {
            miu_k = miu_hat;
            sigma_k = sigma_hat;
        }

        // RCLCPP_INFO(this->get_logger(), "miu_hat = [%.2f, %.2f, %.2f]", miu_hat(0), miu_hat(1), miu_hat(2));
        RCLCPP_INFO(this->get_logger(), "miu_k = [%.2f, %.2f, %.2f]", miu_k(0), miu_k(1), miu_k(2));

        //kalman local end
        //------------------------------------------------------------------------------
        
        quat.setRPY(0, 0, miu_k(2));
        odometry_msg.pose.pose.position.x = miu_k(0);
        odometry_msg.pose.pose.position.y = miu_k(1);
        tf2::convert(quat, odometry_msg.pose.pose.orientation);

        odometry_msg.twist.twist.linear.x = twist_x;
        odometry_msg.twist.twist.angular.z = twist_z;

        odometry_msg.pose.covariance[0]  = sigma_k(0, 0);  
        odometry_msg.pose.covariance[1]  = sigma_k(0, 1);  
        odometry_msg.pose.covariance[5]  = sigma_k(0, 2);  
        odometry_msg.pose.covariance[6]  = sigma_k(1, 0);  
        odometry_msg.pose.covariance[7]  = sigma_k(1, 1);  
        odometry_msg.pose.covariance[11] = sigma_k(1, 2);  
        odometry_msg.pose.covariance[30] = sigma_k(2, 0);  
        odometry_msg.pose.covariance[31] = sigma_k(2, 1);  
        odometry_msg.pose.covariance[35] = sigma_k(2, 2); 

        // Set and publish ROS msgs
        pose_msg.x = miu_k(0);
        pose_msg.y = miu_k(1);
        pose_msg.z = miu_k(2);

        vel_msg.x = miu_hat_dot(0);
        vel_msg.y = miu_hat_dot(1);
        vel_msg.z = miu_hat_dot(2);
        
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


    // -------------------------------------------------------------------------------------------------------------------------
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

        bearing = std::fmod(bearing + M_PI, 2*M_PI) - M_PI;

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
        Z_k = G_k * sigma_hat * G_k.transpose() + R_k;
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
    // -------------------------------------------------------------------------------------------------------------------------
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimateNode>());
    rclcpp::shutdown();
    return 0;
}