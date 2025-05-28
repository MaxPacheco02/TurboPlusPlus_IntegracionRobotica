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
  #include "geometry_msgs/msg/pose_array.hpp"


  #include "aru_interfaces/msg/aruco_list.hpp"


  class FramePublisher : public rclcpp::Node
  {
  public:
    FramePublisher()
    : Node("pzb_tf2_publisher")
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "pzb/odom", 10,
          std::bind(&FramePublisher::handle_pzb_pose, this, std::placeholders::_1));

      pose_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
          "pzb/pose_path", 10);

      aru_sub_ = this->create_subscription<aru_interfaces::msg::ArucoList>(
          "pzb/aru_sys",10,
          std::bind(&FramePublisher::handle_aru_sys, this, std::placeholders::_1));

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

    void handle_aru_sys(const std::shared_ptr<aru_interfaces::msg::ArucoList> msg)
    {
      geometry_msgs::msg::TransformStamped t_cam;
      geometry_msgs::msg::TransformStamped t_aru;

      // ARU SYSTEM BROADCAST
      // BASE -> CAMERA 
      t_cam.header.stamp = this->get_clock()->now();
      t_cam.header.frame_id = "base_link";
      t_cam.child_frame_id = "camera";

      t_cam.transform.translation.x = msg->markers[0].pose.position.x;
      t_cam.transform.translation.y = msg->markers[0].pose.position.y;
      t_cam.transform.translation.z = msg->markers[0].pose.position.z;

      t_cam.transform.rotation.w = msg->markers[0].pose.orientation.w;
      t_cam.transform.rotation.x = msg->markers[0].pose.orientation.x;
      t_cam.transform.rotation.y = msg->markers[0].pose.orientation.y;
      t_cam.transform.rotation.z = msg->markers[0].pose.orientation.z;

      tf_broadcaster_->sendTransform(t_cam); 


      t_aru.header.stamp = this->get_clock()->now();
      t_aru.header.frame_id = "camera";
      t_aru.child_frame_id = "aruco_center";
      // REF -> ARUCO CENTER
      // HACIENDO ESTO ARUCO CENTER SIEMPRE SERA EL MAS CERCANO AL PZB
      if(msg->markers.size() > 1){
        t_aru.transform.translation.x = msg->markers[1].pose.position.x;
        t_aru.transform.translation.y = msg->markers[1].pose.position.y;
        t_aru.transform.translation.z = msg->markers[1].pose.position.z;

        t_aru.transform.rotation.w = msg->markers[1].pose.orientation.w;
        t_aru.transform.rotation.x = msg->markers[1].pose.orientation.x;
        t_aru.transform.rotation.y = msg->markers[1].pose.orientation.y;
        t_aru.transform.rotation.z = msg->markers[1].pose.orientation.z;
      }
      tf_broadcaster_->sendTransform(t_aru);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<aru_interfaces::msg::ArucoList>::SharedPtr aru_sub_;
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