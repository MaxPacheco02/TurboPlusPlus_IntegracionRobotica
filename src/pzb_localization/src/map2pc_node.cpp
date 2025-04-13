#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>

using namespace std::chrono_literals;

class Map2PCNode : public rclcpp::Node
{
public:
    Map2PCNode() : Node("map2pc_node")
    {
        using namespace std::placeholders;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pzb/pose", 10,
            [this](const nav_msgs::msg::Odometry &msg){
                pose = msg.pose.pose;
            });

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/pzb/map", 10,
            [this](const nav_msgs::msg::OccupancyGrid &msg){
            });
            
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pzb/p2", 10);

        timer_=this->create_wall_timer(100ms,std::bind(&Map2PCNode::update,this));

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
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::Pose pose;

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
    rclcpp::spin(std::make_shared<Map2PCNode>());
    rclcpp::shutdown();
    return 0;
}
