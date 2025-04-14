#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "geometry_msgs/msg/vector3.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <thread>
#include <chrono>

#include "include/robot.h"
#include "include/MCL.h"
#include "include/utility.h"
#include "include/world.h"
#include "include/robot.cpp"
#include "include/MCL.cpp"
#include "include/utility.cpp"
#include "include/world.cpp"
#include <iostream>


using namespace std::chrono_literals;

class MCLNode : public rclcpp::Node
{
public:
    MCLNode() : Node("mcl_node")
    {
        using namespace std::placeholders;

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist &msg){
                u.forward = msg.linear.x;
                u.turn = msg.angular.z;

                // move the robot
                R.move(u.turn, u.forward);
                // get sensor measurements
                std::vector<double> z = R.sense(true);
                std::cout << R.get_pose() << std::endl;
                std::cout << R.get_sensor_readings() << std::endl;
                // mcl
                std::vector<particle> belief = MCL(p, u, z).resample();
                std::cout << "[Error]" << utility::evaluation(&R, &belief, &w) << std::endl;
                std::cout << "Particle 0 pos: " << std::endl;
                utility::p0_pos(&R, &belief, &w);
                std::cout << "Particle size: " << p.size() << std::endl;

                
                //set new blief
                utility::visualization(&R, counter++, &p, &belief, &w);
                p = belief;
            });

        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mcl/odom", 10);

        timer_=this->create_wall_timer(10ms,std::bind(&MCLNode::update,this));

        odometry_msg.header.frame_id = "odom";

        // for(int i = 0 ; i < 10 ; i++){
        //     landmarks[2*i] = 20 + 5*i;
        //     landmarks[2*i+1] = 10;
        // }
        // for(int i = 0 ; i < 10 ; i++){
        //     landmarks[20+2*i] = 75;
        //     landmarks[20+2*i+1] = 10+6*i;
        // }
        // for(int i = 0 ; i < 10 ; i++){
        //     landmarks[40+2*i] = 15+6*i;
        //     landmarks[40+2*i+1] = 10+6*i;
        // }     

        w = World(100, 100);

        // set world landmarks
        for (int i = 1; i < sizeof(landmarks) / sizeof(landmarks[0]); i += 2)
        {
            w.set_landmarks(landmarks[i - 1], landmarks[i]);
        }

        R = Robot(w);

        // set robot noise
        R.set_noise(0.05, 0.01, 1.0);

        // std::cout << R.get_pose() << std::endl;

        // u.forward = 1.0;
        // u.turn = -M_PI / 100.0;

        // // move the robot
        // R.move(u.turn, u.forward);
        // std::cout << R.get_pose() << std::endl;
        // std::cout << R.get_sensor_readings() << std::endl;
    
        // first belief; create particle set
        for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
        {
            p.emplace_back(Robot(w), 0.0);
        }
    }

    void update(){
        quat.setRPY(0, 0, R.get_theta());
        odometry_msg.pose.pose.position.x = R.get_x();
        odometry_msg.pose.pose.position.y = R.get_y();
        tf2::convert(quat, odometry_msg.pose.pose.orientation);

        odometry_pub_->publish(odometry_msg);
    }

    private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    nav_msgs::msg::Odometry odometry_msg;

    rclcpp::TimerBase::SharedPtr timer_;

    tf2::Quaternion quat;

    World w;
    // double landmarks[60];
    double landmarks[6]{20,20,40,10,80,90};
    command u;

    Robot R;
    int NUMBER_OF_PARTICLES{1000};
    std::vector<particle> p;

    int counter{0};
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MCLNode>());
    rclcpp::shutdown();
    return 0;
}
