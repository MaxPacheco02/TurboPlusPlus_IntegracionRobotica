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

#include "pzb_msgs/msg/aruco_list.hpp" 

using namespace std::chrono_literals;

class DeadReckoingNode : public rclcpp::Node
{
public:
    DeadReckoingNode() : Node("dead_reckoning"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        using namespace std::placeholders;
        ns_ = this->get_namespace();

        l_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/VelocityEncL", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const std_msgs::msg::Float32 &msg){
                w1 = msg.data;
                update_xid_params_ws();
            });

        r_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/VelocityEncR", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const std_msgs::msg::Float32 &msg){
                w2 = msg.data;
                update_xid_params_ws();
            });

        aru_sub_ = this->create_subscription<pzb_msgs::msg::ArucoList>(
            "pzb/aru_sys", 10,
            std::bind(&DeadReckoingNode::aru_sys_id, this,_1));


        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        update_timer_ = this->create_wall_timer(10ms, std::bind(&DeadReckoingNode::update, this));

        miu_hat << 0, 0, 0;
        miu_hat_dot << 0, 0, 0;

        // para kalman_localization
        //------------------------------------------------  
        Q_k << 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0,
               0.0, 0.0, 0.0;

        // ID'S 0,1,2, etc.
        aruco_map[1] = Eigen::Vector2f(0.90, -0.16);
        aruco_map[2] = Eigen::Vector2f(-0.20, 0.09);
        aruco_map[3] = Eigen::Vector2f(-0.20, 1.26);
        aruco_map[4] = Eigen::Vector2f(-0.20, 1.95);
        aruco_map[5] = Eigen::Vector2f(1.09, 2.34);
        aruco_map[6] = Eigen::Vector2f(2.03, 2.34);
        aruco_map[7] = Eigen::Vector2f(2.30, 0.87);
        aruco_map[8] = Eigen::Vector2f(2.04, -0.16);
        aruco_map[9] = Eigen::Vector2f(0.69, 1.45);
        aruco_map[10] = Eigen::Vector2f(0.73, 0.48);
        aruco_map[11] = Eigen::Vector2f(1.40, 0.77);
        aruco_map[12] = Eigen::Vector2f(0.63, 0.90);
        aruco_map[13] = Eigen::Vector2f(1.73, 1.40);

        I.setIdentity();
        I_2.setIdentity();

        R_k << 0.1, 0.0,
               0.0, 0.02;

        nearest_id = -2;
        z_k << 0.0, 0.0;
        
        H_k.setIdentity();
        sigma_hat.setZero();
        sigma_k.setZero();
        sigma_w.setZero();
        miu_k.setZero();
        delta_w.setZero();
        h_w.setZero();
        z_k.setZero();
        z_hat.setZero();
        Z_k.setZero();
        G_k.setZero();
        K_k.setZero(); 
        //------------------------------------------------

        odometry_msg.header.frame_id = "odom";
    }

private:

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr l_enc_sub_, r_enc_sub_;
    rclcpp::Subscription<pzb_msgs::msg::ArucoList>::SharedPtr aru_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    rclcpp::TimerBase::SharedPtr update_timer_;

    nav_msgs::msg::Odometry odometry_msg;

    tf2::Quaternion quat;

    std::string ns_;

    double w1{0.0}, w2{0.0};
    double twist_x{0.0}, twist_z{0.0};

    const double r{0.05}, l{0.08}, dt{0.01};

    //------------------------------------------------
    // Para Kalman
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::msg::TransformStamped tf_aru;
    
    std::unordered_map<int, Eigen::Vector2f> aruco_map;

    Eigen::Vector3f miu_k, miu_hat, miu_hat_dot;
    Eigen::Vector2f z_k, z_hat;

    Eigen::Matrix3f sigma_k, sigma_hat, H_k, Q_k, I;
    Eigen::Matrix2f sigma_w, Z_k, R_k, I_2;

    Eigen::Matrix<float, 2, 3> G_k;
    Eigen::Matrix<float, 3, 2> K_k, h_w, delta_w;

    int nearest_id;
    //------------------------------------------------

    void update() {      
        // RCLCPP_ERROR(get_logger(), "%f,%f,%f - %f, %f", xi(0),xi(1),xi(2), twist_x, twist_z);

        miu_hat_dot << twist_x* cos(miu_k(2)),
                       twist_x* sin(miu_k(2)),
                       twist_z;
        
        // Integrate
        miu_hat << miu_k + miu_hat_dot * dt;

        // Make theta [-pi, pi]
        miu_hat(2) = std::fmod(miu_hat(2) + M_PI, 2*M_PI) - M_PI;

        std::cout << "miu_hat: " << miu_hat << "\n" <<std::endl;

        //kalman local start
        H_k(0,2) = -(dt * miu_hat_dot(1));
        H_k(1,2) = dt * miu_hat_dot(0);
        
        sigma_w(0,0) = 0.05 * std::abs(w2); //kr
        sigma_w(1,1) = 0.05 * std::abs(w1); //kl

        delta_w << cos(miu_k(2)), cos(miu_k(2)),
                   sin(miu_k(2)), sin(miu_k(2)),
                   2/l, -2/l;

        h_w = 0.5 * r * dt * delta_w;
        Q_k = h_w * sigma_w * h_w.transpose();

        sigma_hat = (H_k * sigma_k * H_k.transpose()) + Q_k;

        auto it = aruco_map.find(nearest_id);

        // REVISAR SI EL ID EXISTE EN EL MAPA 
        if (it != aruco_map.end()) { //&& z_k(1) != 0.0
            const auto& marker_pos = it->second;
            calc_zHat(marker_pos.x(), marker_pos.y());
            calc_Gradient_G(marker_pos.x(), marker_pos.y());
            everything_else();
        } else {
            miu_k = miu_hat;
            sigma_k = sigma_hat;
        }

        std::cout << "miu_k: " << miu_k << "\n" <<std::endl;

        // Set and publish ROS msgs
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

        odometry_msg.header.stamp = this->get_clock()->now();

        odometry_pub_->publish(odometry_msg);
    }

    // Obtain velocity vector params with wheels' ang. vel
    void update_xid_params_ws(){
        twist_x = (r/2) * (w1 + w2);
        twist_z = (r/(2*l)) * (w2 - w1);
    }

    // -------------------------------------------------------------------------------------------------------------------------
    void aru_sys_id(const pzb_msgs::msg::ArucoList &msg){
        if(msg.markers.size() > 1){
            // CALCULO DE z_k CON MATRICES HOMOGENEAS
            tf2::Quaternion quat;
            quat.setRPY(1.5708f, -M_PI, -M_PI - 1.5708f);
            Eigen::Quaternionf q_cam(quat.w(),quat.x(),quat.y(),quat.z());
            Eigen::Matrix3f Rc = q_cam.toRotationMatrix();

            Eigen::Matrix4f H_T_base = Eigen::Matrix4f::Identity();
            H_T_base.block<3, 3>(0,0) = Rc;
            H_T_base.block<3, 1>(0,3) << 0.108, 0.0, 0.044;
            
            Eigen::Matrix4f H_T_axis = Eigen::Matrix4f::Identity();
            H_T_axis.block<3, 3>(0,0) = Eigen::Matrix3f::Identity();
            H_T_axis.block<3, 1>(0,3) << -0.052, 0.0, 0.049;

            Eigen::Matrix4f H_T_aru_axis = H_T_axis * H_T_base;

            Eigen::Vector4f pos_aru;
            pos_aru << msg.markers[1].pose.position.x, 
                       msg.markers[1].pose.position.y,
                       msg.markers[1].pose.position.z,
                       1.0;
            
            Eigen::Vector4f pos_axis = H_T_aru_axis * pos_aru;
            nearest_id = msg.markers[1].id;

            double alpha = atan2(pos_axis(1),pos_axis(0));

            alpha = std::fmod(alpha + M_PI, 2*M_PI) - M_PI;

            z_k << std::hypot(pos_axis(0), pos_axis(1)), alpha;

        }else{
            nearest_id = -2;
        }
        // rclcpp::Time now = this->get_clock()->now();
        // if (tf_buffer_.canTransform("pzb/base_axis","aruco_center",now,rclcpp::Duration::from_seconds(0.00001))) {
        //     if(msg.markers.size() > 1){
        //         tf_aru = tf_buffer_.lookupTransform("pzb/base_axis","aruco_center",now);
        //         nearest_id = msg.markers[1].id;
        //         double alpha = atan2(tf_aru.transform.translation.y,tf_aru.transform.translation.x);
        //         z_k << std::hypot(tf_aru.transform.translation.x, tf_aru.transform.translation.y), alpha;
        //     }else{
        //         nearest_id = -2;
        //     }
        // }
    }

    void calc_zHat(double m_x_i, double m_y_i) {
        double dx = m_x_i - miu_hat(0);
        double dy = m_y_i - miu_hat(1);

        double range = std::hypot(dx, dy);
        double bearing = std::atan2(dy, dx) - miu_hat(2);

        bearing = std::fmod(bearing + M_PI, 2*M_PI) - M_PI;

        z_hat << range, bearing; //6
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
        G_k(1, 2) = -1.0; //7
    }

    void everything_else(){
        Z_k = G_k * sigma_hat * G_k.transpose() + R_k; //8
        K_k = sigma_hat * G_k.transpose() * Z_k.ldlt().solve(I_2); //9
        miu_k = miu_hat + K_k * (z_k - z_hat); //10
        miu_k(2) = std::fmod(miu_k(2) + M_PI, 2*M_PI) - M_PI;
        // std::cout << "z_k: " << z_k << "\n" <<std::endl;
        // std::cout << "z_hat: " << z_hat << "\n" <<std::endl;
        sigma_k = (I - K_k * G_k) * sigma_hat; //11
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeadReckoingNode>());
    rclcpp::shutdown();
    return 0;
}
