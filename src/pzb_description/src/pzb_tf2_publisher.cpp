#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("pzb_tf2_publisher")
  {
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "pzb/odom", 10,
        std::bind(&FramePublisher::handle_pzb_pose, this, std::placeholders::_1));

    pose_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "pzb/pose_path", 10);

    pose_stamped_tmp_.header.frame_id = "odom";
    pose_path.header.frame_id = "odom";
    pose_path.header.stamp = FramePublisher::now();
  }

private:
  void handle_pzb_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {

    // Update TF2 msg
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_axis";

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    t.transform.rotation.w = msg->pose.pose.orientation.w;
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;

    tf_broadcaster_->sendTransform(t);

    // Update Path msg
    pose_stamped_tmp_.pose.position.x = msg->pose.pose.position.x;
    pose_stamped_tmp_.pose.position.y = msg->pose.pose.position.y;
    pose_path.poses.push_back(pose_stamped_tmp_);
    
    // Erase elements when path is too long
    if(pose_path.poses.size() > 1000){
      pose_path.poses.erase(pose_path.poses.begin(), pose_path.poses.begin()+1);
    }

    pose_path_pub_->publish(pose_path);

  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_path_pub_;
  geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
  nav_msgs::msg::Path pose_path;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}