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

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "tf2_ros/transform_broadcaster.h"

class AruWaypoint : public rclcpp::Node {
    
public:
    AruWaypoint() : Node("AruWaypoint"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_){
        RCLCPP_INFO(this->get_logger(), "DETECCION DE CAJAS");

        using namespace std::placeholders;
        
        pallet_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pzb/pallet_pos",10,
          std::bind(&AruWaypoint::pallet_sys_id, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom",10,
          std::bind(&AruWaypoint::handle_odom, this, _1));

        pallet_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        quat_rot.setRPY(1.5708f,1.5708f,0.0);
        pose_stamped_msg_.header.frame_id = "odom";

        Ra.setZero();
        pos_aru.setZero();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        t_aru.header.frame_id = "odom";
        t_aru.child_frame_id = "aaaaaa";
    }

private:

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_pose_;

    geometry_msgs::msg::PoseStamped pose_stamped_msg_; 

    tf2::Quaternion quat_rot;
    tf2::Quaternion quat;

    geometry_msgs::msg::TransformStamped tf_aru;

    nav_msgs::msg::Odometry odom_pos;

    Eigen::Matrix4f H_T_base;
    Eigen::Matrix3f Ra;
    Eigen::Vector4f pos_aru;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped t_aru;

    void handle_odom(const nav_msgs::msg::Odometry &msg){
        
        pose_stamped_msg_.header.stamp = this->get_clock()->now();

        Eigen::Vector3f pos_odom;
        pos_odom << msg.pose.pose.position.x, 
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z;

        Eigen::Quaternionf q_odom(msg.pose.pose.orientation.w,
                                 msg.pose.pose.orientation.x,
                                 msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z);
        Eigen::Matrix3f Ro = q_odom.toRotationMatrix();

        Eigen::Matrix4f H_T_odom = Eigen::Matrix4f::Identity();
        H_T_odom.block<3, 3>(0,0) = Ro;
        H_T_odom.block<3, 1>(0,3) = pos_odom;

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

        Eigen::Matrix4f H_T_aru_odom = H_T_odom * H_T_axis * H_T_base;
        Eigen::Vector4f pos_aru_odom = H_T_aru_odom * pos_aru;
        Eigen::Matrix3f R_aru_odom = Ro*Rc*Ra;
        
        Eigen::Quaternionf qa(R_aru_odom);
        
        pose_stamped_msg_.pose.position.x = pos_aru_odom[0];
        pose_stamped_msg_.pose.position.y = pos_aru_odom[1];
        pose_stamped_msg_.pose.position.z = pos_aru_odom[2];
        
        pose_stamped_msg_.pose.orientation.x = qa.x();
        pose_stamped_msg_.pose.orientation.y = qa.y();
        pose_stamped_msg_.pose.orientation.z = qa.z();
        pose_stamped_msg_.pose.orientation.w = qa.w();
        
        pallet_pose_->publish(pose_stamped_msg_);

        t_aru.header.stamp = this->get_clock()->now();

        t_aru.transform.translation.x = pos_aru_odom[0];
        t_aru.transform.translation.y = pos_aru_odom[1];
        t_aru.transform.translation.z = pos_aru_odom[2];

        t_aru.transform.rotation.w = qa.w();
        t_aru.transform.rotation.x = qa.x();
        t_aru.transform.rotation.y = qa.y();
        t_aru.transform.rotation.z = qa.z();

        tf_broadcaster_->sendTransform(t_aru);
        
    }
    

    void pallet_sys_id(const geometry_msgs::msg::PoseStamped &msg){
        pos_aru << msg.pose.position.x, 
                   msg.pose.position.y,
                   msg.pose.position.z,
                   1.0;

        quat = tf2::Quaternion(msg.pose.orientation.x, 
                                msg.pose.orientation.y,     
                                msg.pose.orientation.z,     
                                msg.pose.orientation.w);

        tf2::Quaternion quat_res = quat_rot * quat;
        quat_res.normalize();
        Eigen::Quaternionf q_cam(quat_res.w(),quat_res.x(),quat_res.y(),quat_res.z());
        Ra = q_cam.toRotationMatrix(); 

        // if (tf_buffer_.canTransform("odom","pallet_aru",this->get_clock()->now(),rclcpp::Duration::from_seconds(0.0001))) {
        //     tf_aru = tf_buffer_.lookupTransform("odom","pallet_aru",this->get_clock()->now());
        //     pose_stamped_msg_.pose.position.x = tf_aru.transform.rotation.x;
        //     pose_stamped_msg_.pose.position.y = tf_aru.transform.rotation.y;
        //     pose_stamped_msg_.pose.position.z = tf_aru.transform.rotation.z;

        //     quat = tf2::Quaternion(tf_aru.transform.rotation.x, 
        //                            tf_aru.transform.rotation.y,     
        //                            tf_aru.transform.rotation.z,     
        //                            tf_aru.transform.rotation.w);

        //     tf2::Quaternion quat_res = quat_rot * quat;
        //     quat_res.normalize();

        //     pose_stamped_msg_.pose.orientation.x = quat_res.x();
        //     pose_stamped_msg_.pose.orientation.y = quat_res.y();
        //     pose_stamped_msg_.pose.orientation.z = quat_res.z();
        //     pose_stamped_msg_.pose.orientation.w = quat_res.w();

        //     pallet_pose_->publish(pose_stamped_msg_);

        // }
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AruWaypoint>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 1;
}
