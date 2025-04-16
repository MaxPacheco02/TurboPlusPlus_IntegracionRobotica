#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

#include <chrono>
#include <string>
#include <array>

#include "include/map_utils.h"
#include "include/map_utils.cpp"

#include "include/robot.h"
#include "include/MCL.h"
#include "include/utility.h"

#include "include/robot.cpp"
#include "include/MCL.cpp"
#include "include/utility.cpp"

using namespace std::chrono_literals;

class MCLNode : public rclcpp::Node
{
public:
    MCLNode() : Node("mcl_node")
    {
        using namespace std::placeholders;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pzb/odom", 10,
            [this](const nav_msgs::msg::Odometry &msg){
                pose = msg.pose.pose;                
            });

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/pzb/map", 10,
            [this](const nav_msgs::msg::OccupancyGrid &msg){
                if(grid_map.data.size() == msg.data.size())
                    return;

                grid_map = msg;
                map = map2vec(grid_map);
                fill_pc(map_pc_msg, map, 0, 0, 255, 255, 0.0);
            });
            
        map_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pzb/map_pc", 10);
        local_map_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pzb/pc", 10);
        mcl_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mcl/odom", 10);
        cmd_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mcl/command_odom", 10);

        timer_=this->create_wall_timer(20ms,std::bind(&MCLNode::update,this));

        // Initialize PointCloud msgs
        init_pc_msg(map_pc_msg);
        init_pc_msg(local_map_pc_msg);

        // Initialize MCL Odometry msg
        mcl_msg.header.frame_id = "odom";
        cmd_odom_msg.header.frame_id = "odom";

        // Setup robot (MCL) variables
        robot.set_states(0.0, 0.0, 0.0);
        robot.set_noise(0.05, 0.01, 1.0);

        for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
        {
            particles.emplace_back(Robot(), 0.0);
        }

        pose.position = geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0);
        pose.orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1);

    }

    void update(){
        mcl();

        map_pc_msg.header.stamp = this->get_clock()->now();
        local_map_pc_msg.header.stamp = this->get_clock()->now();
        mcl_msg.header.stamp = this->get_clock()->now();
        cmd_odom_msg.header.stamp = this->get_clock()->now();

        map_pc_pub_->publish(map_pc_msg);
        local_map_pc_pub_->publish(local_map_pc_msg);

        mcl_pub_->publish(mcl_msg);
        cmd_odom_pub_->publish(cmd_odom_msg);

        // std::cout << robot.get_pose() << std::endl;
    }

    void mcl(){

        // Update raycast pc2 msg
        compute_raycast(map, local_map, pose, 9.0);
        fill_pc(local_map_pc_msg, local_map, 255, 0, 0, 255, 0.05);

        // Calculate linear displacement in global frame
        double dx = pose.position.x - last_pose.position.x;
        double dy = pose.position.y - last_pose.position.y;
        
        // Get yaw angles from quaternions
        double yaw = tf2::getYaw(pose.orientation);
        double yaw_last = tf2::getYaw(last_pose.orientation);
        
        // Calculate forward motion in robot's local frame
        // Project the displacement onto the robot's previous heading
        control.forward = std::cos(yaw_last) * dx + std::sin(yaw_last) * dy;
        
        // Calculate turning angle (handle wrapping around +/- PI)
        control.turn = yaw - yaw_last;
        if (control.turn > M_PI) control.turn -= 2 * M_PI;
        if (control.turn < -M_PI) control.turn += 2 * M_PI;

        // Execute movement (with noise)
        // robot.move(control.turn, control.forward);

        // Read sensor
        std::vector<double> z = robot.sense(map, pose, true);

        // Run MCL
        std::vector<particle> belief = MCL(particles, control, z, map).resample();
        
        // Visualize result
        utility::visualization(&robot, 0, &particles, &belief, map);

        // Update particles
        particles = belief;

        cmd_odom_msg.pose.pose = robot.get_pose_msg();

        last_pose = pose;

    }
    
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_pub_, local_map_pc_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr cmd_odom_pub_, mcl_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::Pose last_pose, pose;

    nav_msgs::msg::OccupancyGrid grid_map;
    nav_msgs::msg::Odometry mcl_msg, cmd_odom_msg;
    
    sensor_msgs::msg::PointCloud2 map_pc_msg;
    sensor_msgs::msg::PointCloud2 local_map_pc_msg;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Point> map, local_map;

    Robot robot;
    command control;
    std::vector<particle> particles;
    int NUMBER_OF_PARTICLES = 1000;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MCLNode>());
    rclcpp::shutdown();
    return 0;
}
