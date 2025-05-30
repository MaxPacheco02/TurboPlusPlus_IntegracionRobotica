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

#include "aru_interfaces/msg/aruco_list.hpp"  

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class AruWaypoint : public rclcpp::Node {
    
public:
    AruWaypoint() : Node("ObjPos"){
        RCLCPP_INFO(this->get_logger(), "DETECCION DE CAJAS");

        using namespace std::placeholders;
        
        pallet_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pzb/palway", 10);

        aru_sub_ = this->create_subscription<aru_interfaces::msg::ArucoList>(
            "pzb/aru_sys", 10,
            std::bind(&AruWaypoint::aru_sys_id, this,_1));
    }

private:

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_pose_;
    rclcpp::Subscription<aru_interfaces::msg::ArucoList>::SharedPtr aru_sub_;

    geometry_msgs::msg::PoseStamped pose_stamped_msg_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped tf_aru;
    
    int nearest_id;
    
    void aru_sys_id(const aru_interfaces::msg::ArucoList &msg){
        rclcpp::Time now = this->get_clock()->now();
        if (tf_buffer_.canTransform("odom","aruco_center",now,rclcpp::Duration::from_seconds(0.001))) {
            if(msg.markers.size() > 1){
                tf_aru = tf_buffer_.lookupTransform("odom","aruco_center",now);
                nearest_id = msg.markers[1].id;
                double alpha = atan2(tf_aru.transform.translation.y,tf_aru.transform.translation.x);
                z_k << std::hypot(tf_aru.transform.translation.x, tf_aru.transform.translation.y), alpha;
            }else{
                nearest_id = -2;
            }
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
