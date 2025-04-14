#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <string>
#include <array>

#include "map_utils.h"
#include "map_utils.cpp"

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
                compute_raycast(map, local_map, pose, 4.0);
                fill_pc(local_map_pc_msg, local_map, 255, 0, 0, 255, 0.05);                        
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

        timer_=this->create_wall_timer(100ms,std::bind(&MCLNode::update,this));

        init_pc_msg(map_pc_msg);
        init_pc_msg(local_map_pc_msg);
    }

    void update(){
        map_pc_msg.header.stamp = this->get_clock()->now();
        local_map_pc_msg.header.stamp = this->get_clock()->now();

        map_pc_pub_->publish(map_pc_msg);
        local_map_pc_pub_->publish(local_map_pc_msg);
    }
    
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_pub_, local_map_pc_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::Pose pose;

    nav_msgs::msg::OccupancyGrid grid_map;
    
    sensor_msgs::msg::PointCloud2 map_pc_msg;
    sensor_msgs::msg::PointCloud2 local_map_pc_msg;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Point> map, local_map;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MCLNode>());
    rclcpp::shutdown();
    return 0;
}
