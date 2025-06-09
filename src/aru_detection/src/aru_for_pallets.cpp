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

class AruWaypoint : public rclcpp::Node {
    
public:
    AruWaypoint() : Node("AruWaypoint"){
        RCLCPP_INFO(this->get_logger(), "DETECCION DE CAJAS");

        using namespace std::placeholders;

        pallet_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pallet/goal_pose", 10);

        H_T_base_axis.setIdentity();
        
        // MATRIZ HOMOGENEA DE CAMARA *20 deg tilt downward*
        quat.setRPY(1.5708f - 0.34806f, -M_PI, -M_PI - 1.5708f); //- 0.34806f
        Eigen::Quaternionf q_cam(quat.w(),quat.x(),quat.y(),quat.z());
        Rc = q_cam.toRotationMatrix();
        H_T_base_axis.block<3, 3>(0,0) = Rc;
        H_T_base_axis.block<3, 1>(0,3) << 0.09, 0.0, 0.225;
        
        // PARA CORRECION DE ROTACION DE ARUCO
        R_corr_aru = Eigen::AngleAxisf(roll_correction,  Eigen::Vector3f::UnitX()) *
                    Eigen::AngleAxisf(yaw_correction, Eigen::Vector3f::UnitZ());
        R_corr_odom = Eigen::AngleAxisf(yaw_correction_odom, Eigen::Vector3f::UnitZ());

        pallet_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pzb/pallet_pos", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
          std::bind(&AruWaypoint::pallet_sys_id, this, _1));

        pose_stamped_msg_.header.frame_id = "base_axis";
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_pose_;

    geometry_msgs::msg::PoseStamped pose_stamped_msg_; 

    tf2::Quaternion quat;

    Eigen::Matrix3f Ra, Rc, R_aru_odom, R_corr_aru, R_corr_odom, R, R_final;
    Eigen::Vector4f pos_aru, pos_aru_odom;
    Eigen::Vector3f pos_odom;

    Eigen::Matrix4f H_T_base_axis;

    float roll_correction{-M_PI / 2.0f};  
    float yaw_correction{-M_PI / 2.0f};
    float yaw_correction_odom{M_PI}; 

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
        
        pos_aru_odom.head<3>() = H_T_base_axis.block<3,3>(0,0) * pos_aru.head<3>() + H_T_base_axis.block<3,1>(0,3); //pos_aru_odom = H_T_base_axis * pos_aru;
    
        R_aru_odom = Rc*Ra;
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
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AruWaypoint>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}