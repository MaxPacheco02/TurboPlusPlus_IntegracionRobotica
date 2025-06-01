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

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class AruWaypoint : public rclcpp::Node {
    
public:
    AruWaypoint() : Node("ObjPos"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_){
        RCLCPP_INFO(this->get_logger(), "DETECCION DE CAJAS");

        using namespace std::placeholders;
        
        pallet_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pzb/pallet_pos", 10);

        quat_rot.setRPY(0.0,M_PI,0.0);
        pose_stamped_msg_.header.frame_id = "odom";
    }

private:

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_pose_;

    geometry_msgs::msg::PoseStamped pose_stamped_msg_;

    tf2::Quaternion quat_rot;
    tf2::Quaternion quat;

    geometry_msgs::msg::TransformStamped tf_aru;
    
    int nearest_id;
    
    void pallet_sys_id(const geometry_msgs::msg::PoseStamped &msg){
        if (tf_buffer_.canTransform("odom","pallet_aru",this->get_clock()->now(),rclcpp::Duration::from_seconds(0.001))) {
            tf_aru = tf_buffer_.lookupTransform("odom","pallet_aru",this->get_clock()->now());   
            pose_stamped_msg_.pose.position.x = msg.pose.position.x;
            pose_stamped_msg_.pose.position.y = msg.pose.position.y;
            pose_stamped_msg_.pose.position.z = msg.pose.position.z;

            quat = tf2::Quaternion(msg.pose.orientation.x, 
                                   msg.pose.orientation.y,     
                                   msg.pose.orientation.z,     
                                   msg.pose.orientation.w);

            tf2::Quaternion quat_res = quat_rot * quat;
            quat_res.normalize();

            pose_stamped_msg_.pose.orientation.x = quat_res.x();
            pose_stamped_msg_.pose.orientation.y = quat_res.y();
            pose_stamped_msg_.pose.orientation.z = quat_res.z();
            pose_stamped_msg_.pose.orientation.w = quat_res.w();

            pallet_pose_->publish(pose_stamped_msg_);
        }
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AruWaypoint>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 1;
}
