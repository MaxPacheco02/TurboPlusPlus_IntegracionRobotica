#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <cmath>
#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include <tf2/utils.h>

#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/core/eigen.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_broadcaster.h"

class AruWaypoint : public rclcpp::Node {
    
public:
    AruWaypoint() : Node("AruWaypoint"){
        RCLCPP_INFO(this->get_logger(), "DETECCION DE CAJAS");

        using namespace std::placeholders;

        pallet_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        H_T_odom.setIdentity();
        H_T_base_axis.setIdentity();
        H_T_aru_odom.setIdentity();
        
        // MATRIZ HOMOGENEA DE CAMARA
        quat.setRPY(1.5708f, -M_PI, -M_PI - 1.5708f);
        Eigen::Quaternionf q_cam(quat.w(),quat.x(),quat.y(),quat.z());
        Rc = q_cam.toRotationMatrix();
        H_T_base_axis.block<3, 3>(0,0) = Rc;
        H_T_base_axis.block<3, 1>(0,3) << 0.056, 0.0, 0.153;
        
        // PARA CORRECION DE ROTACION DE ARUCO
        R_corr_aru = Eigen::AngleAxisf(roll_correction,  Eigen::Vector3f::UnitX()) *
                    Eigen::AngleAxisf(yaw_correction, Eigen::Vector3f::UnitZ());
        R_corr_odom = Eigen::AngleAxisf(yaw_correction_odom, Eigen::Vector3f::UnitZ());

        pallet_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pzb/pallet_pos", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
          std::bind(&AruWaypoint::pallet_sys_id, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
          std::bind(&AruWaypoint::handle_odom, this, _1));

        pose_stamped_msg_.header.frame_id = "odom";

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_pose_;

    geometry_msgs::msg::PoseStamped pose_stamped_msg_; 

    tf2::Quaternion quat;

    Eigen::Matrix3f Ra, Ro, Rc, R_aru_odom, R_corr_aru, R_corr_odom, R, R_final;
    Eigen::Vector4f pos_aru, pos_aru_odom;
    Eigen::Vector3f pos_odom;

    Eigen::Matrix4f H_T_odom, H_T_base_axis, H_T_aru_odom;

    float roll_correction{-M_PI / 2.0f};  
    float yaw_correction{-M_PI / 2.0f};
    float yaw_correction_odom{M_PI}; 


    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped t_aru;

    void handle_odom(const nav_msgs::msg::Odometry &msg){
        pos_odom << msg.pose.pose.position.x, 
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z;

        Eigen::Quaternionf q_odom(msg.pose.pose.orientation.w,
                                 msg.pose.pose.orientation.x,
                                 msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z);
        Ro = q_odom.toRotationMatrix();

        H_T_odom.block<3, 3>(0,0) = Ro;
        H_T_odom.block<3, 1>(0,3) = pos_odom;
    }
    

    void pallet_sys_id(const geometry_msgs::msg::PoseStamped &msg){
        pos_aru << msg.pose.position.x, 
                   msg.pose.position.y,
                   msg.pose.position.z,
                   1.0;
        Eigen::Quaternionf qa(msg.pose.orientation.w, 
                            msg.pose.orientation.x,     
                            msg.pose.orientation.y,     
                            msg.pose.orientation.z);           

        R = qa.toRotationMatrix();
        R_final = R * R_corr_aru;
        Ra = R_final;

        H_T_aru_odom = H_T_odom * H_T_base_axis;
        
        pos_aru_odom.head<3>() = H_T_aru_odom.block<3,3>(0,0) * pos_aru.head<3>() + H_T_aru_odom.block<3,1>(0,3); //pos_aru_odom = H_T_aru_odom * pos_aru;
        R_aru_odom = Ro*Rc*Ra;

        R_final = R_aru_odom * R_corr_odom;
        
        Eigen::Quaternionf q(R_final);
        pose_stamped_msg_.header.stamp = msg.header.stamp;
        pose_stamped_msg_.pose.position.x = pos_aru_odom[0];
        pose_stamped_msg_.pose.position.y = pos_aru_odom[1];
        pose_stamped_msg_.pose.position.z = pos_aru_odom[2];
        
        pose_stamped_msg_.pose.orientation.x = q.x();
        pose_stamped_msg_.pose.orientation.y = q.y();
        pose_stamped_msg_.pose.orientation.z = q.z();
        pose_stamped_msg_.pose.orientation.w = q.w();

        pallet_pose_->publish(pose_stamped_msg_);

        t_aru.header.frame_id = "odom";
        t_aru.child_frame_id = "waypoint";
        t_aru.header.stamp = msg.header.stamp;
        t_aru.transform.translation.x = pos_aru_odom[0];
        t_aru.transform.translation.y = pos_aru_odom[1];
        t_aru.transform.translation.z = pos_aru_odom[2];

        t_aru.transform.rotation.w = q.w();
        t_aru.transform.rotation.x = q.x();
        t_aru.transform.rotation.y = q.y();
        t_aru.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(t_aru);
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AruWaypoint>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}