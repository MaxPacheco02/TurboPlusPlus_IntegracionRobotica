#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <cmath>
#include <tf2/utils.h>
#include <string>
#include <vector>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "pzb_msgs/msg/aruco_list.hpp"

class AruPublisher : public rclcpp::Node {
public:
    AruPublisher() : Node("aruco_tf_publisher") {

        using namespace std::placeholders;
        
        this->declare_parameter("parent_frame", "odom");
        std::string parent_frame = this->get_parameter("parent_frame").as_string();
        std::vector<std::string> param_names = {"id1", "id2", "id3", "id4", "id5", "id6", "id7", 
                                                "id8", "id9", "id10", "id11", "id12", "id13"};

        for(int i = 0; i < param_names.size(); i++){
            declare_marker_params(param_names[i]);
        }

        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        for(unsigned long int i = 0; i < param_names.size(); i++){
            std::string id = param_names[i];
        
            std::vector<double> pos = this->get_parameter(id+".position").as_double_array();
            double yaw = this->get_parameter(id+".yaw").as_double();
        
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id = id; 
            tf_msg.header.stamp = this->get_clock()->now();
        
            tf_msg.transform.translation.x = pos[0];
            tf_msg.transform.translation.y = pos[1];
            tf_msg.transform.translation.z = pos[2];
    
            q.setRPY(0.0, 0.0, yaw);
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_msg.transform.rotation.w = q.w();

            transforms.push_back(tf_msg);
        }
        static_broadcaster_->sendTransform(transforms);


        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        aru_sub_ = this->create_subscription<pzb_msgs::msg::ArucoList>(
          "pzb/aru_sys",10,
          std::bind(&AruPublisher::handle_aru_sys, this, _1));
        pallet_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "pzb/pallet_pos",10,
          std::bind(&AruPublisher::handle_pallet_sys, this, _1));

        t_cam.header.frame_id = "pzb/base_link";
        t_cam.child_frame_id = "camera";
        t_aru.header.frame_id = "camera";
        t_aru.child_frame_id = "aruco_center";
        t_pal.header.frame_id = "camera";
        t_pal.child_frame_id = "pallet_aru";
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    geometry_msgs::msg::TransformStamped tf_msg;
    tf2::Quaternion q;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<pzb_msgs::msg::ArucoList>::SharedPtr aru_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_sub_;

    geometry_msgs::msg::TransformStamped t_cam;
    geometry_msgs::msg::TransformStamped t_aru;
    geometry_msgs::msg::TransformStamped t_pal;
    

    void declare_marker_params(const std::string& id) {
        this->declare_parameter<std::vector<double>>(id + ".position");
        this->declare_parameter<double>(id + ".yaw");
    }

    void handle_aru_sys(const std::shared_ptr<pzb_msgs::msg::ArucoList> msg){
        // ARU SYSTEM BROADCAST
        // BASE -> CAMERA 
        t_cam.header.stamp = this->get_clock()->now();

        t_cam.transform.translation.x = msg->markers[0].pose.position.x;
        t_cam.transform.translation.y = msg->markers[0].pose.position.y;
        t_cam.transform.translation.z = msg->markers[0].pose.position.z;

        t_cam.transform.rotation.w = msg->markers[0].pose.orientation.w;
        t_cam.transform.rotation.x = msg->markers[0].pose.orientation.x;
        t_cam.transform.rotation.y = msg->markers[0].pose.orientation.y;
        t_cam.transform.rotation.z = msg->markers[0].pose.orientation.z;

        tf_broadcaster_->sendTransform(t_cam); 

        t_aru.header.stamp = this->get_clock()->now();

        // REF -> ARUCO CENTER
        // ARUCO CENTER SIEMPRE EL MAS CERCANO AL PZB
        if(msg->markers.size() > 1){
            t_aru.transform.translation.x = msg->markers[1].pose.position.x;
            t_aru.transform.translation.y = msg->markers[1].pose.position.y;
            t_aru.transform.translation.z = msg->markers[1].pose.position.z;

            t_aru.transform.rotation.w = msg->markers[1].pose.orientation.w;
            t_aru.transform.rotation.x = msg->markers[1].pose.orientation.x;
            t_aru.transform.rotation.y = msg->markers[1].pose.orientation.y;
            t_aru.transform.rotation.z = msg->markers[1].pose.orientation.z;
        }else{
            t_aru.transform.translation.x = 0.0;
            t_aru.transform.translation.y = 0.0;
            t_aru.transform.translation.z = 0.0;

            t_aru.transform.rotation.w = 1.0;
            t_aru.transform.rotation.x = 0.0;
            t_aru.transform.rotation.y = 0.0;
            t_aru.transform.rotation.z = 0.0;
        }
        tf_broadcaster_->sendTransform(t_aru);
    }

    void handle_pallet_sys(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg){
        t_pal.header.stamp = this->get_clock()->now();
        t_pal.transform.translation.x = msg->pose.position.x;
        t_pal.transform.translation.y = msg->pose.position.y;
        t_pal.transform.translation.z = msg->pose.position.z;
        t_pal.transform.rotation.w = msg->pose.orientation.w;
        t_pal.transform.rotation.x = msg->pose.orientation.x;
        t_pal.transform.rotation.y = msg->pose.orientation.y;
        t_pal.transform.rotation.z = msg->pose.orientation.z;
        tf_broadcaster_->sendTransform(t_pal);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AruPublisher>());
    rclcpp::shutdown();
    return 0;
}
