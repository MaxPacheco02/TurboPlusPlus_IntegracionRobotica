#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono;

class StatePublisher : public rclcpp::Node{
    public:

    StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
        Node("state_publisher",options){
            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);

            pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
              "/pzb_pose", 10,std::bind(&PoseSubscriber::pose_callback, this, std::placeholders::_1));

            broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            RCLCPP_INFO(this->get_logger(),"Starting state publisher");

            loop_rate_=std::make_shared<rclcpp::Rate>(33ms);

            timer_=this->create_wall_timer(33ms,std::bind(&StatePublisher::publish,this));
        }

    void publish();
    void pose_callback();

    private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::Rate::SharedPtr loop_rate_;
    rclcpp::TimerBase::SharedPtr timer_;

    //Robot state variables
    // degree means one degree
    const double degree=M_PI/180.0;
    double angle = 0.0;
    double right_wheel_joint = 0;
    double left_wheel_joint = 0;

};

void StatePublisher::pose_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    // Handle the received message
    RCLCPP_INFO(this->get_logger(), "Received Pose: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
}

void StatePublisher::publish(){
    // create the necessary messages
    geometry_msgs::msg::TransformStamped t;
    sensor_msgs::msg::JointState joint_state;

    // add time stamp
    joint_state.header.stamp=this->get_clock()->now();
    // Specify joints' name which are defined in the r2d2.urdf.xml and their content
    joint_state.name={"right_wheel_joint","left_wheel_joint"};
    joint_state.position={right_wheel_joint,left_wheel_joint};

    // add time stamp
    t.header.stamp=this->get_clock()->now();
    // specify the father and child frame

    // odom is the base coordinate system of tf2
    t.header.frame_id="odom";
    // axis is defined in r2d2.urdf.xml file and it is the base coordinate of model
    t.child_frame_id="base_link";

    // add translation change
    // t.transform.translation.x=cos(angle)*2;
    // t.transform.translation.y=sin(angle)*2;
    t.transform.translation.x=0;
    t.transform.translation.y=0;

    t.transform.translation.z=0.0;

    tf2::Quaternion q;
    // euler angle into Quanternion and add rotation change
    // q.setRPY(0,0,angle+M_PI/2);
    q.setRPY(0,0,0);
    t.transform.rotation.x=q.x();
    t.transform.rotation.y=q.y();
    t.transform.rotation.z=q.z();
    t.transform.rotation.w=q.w();

    // update state for next time
    right_wheel_joint+=degree/2;  // Increment by 1 degree (in radians)
    left_wheel_joint+=degree/2;  // Increment by 1 degree (in radians)
    angle+=degree;    // Change angle at a slower pace

    // send message
    broadcaster->sendTransform(t);
    joint_pub_->publish(joint_state);

    // RCLCPP_INFO(this->get_logger(),"Publishing joint state");
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}
