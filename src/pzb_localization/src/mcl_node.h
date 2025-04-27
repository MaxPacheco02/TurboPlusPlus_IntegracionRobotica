#ifndef MCL_NODE_H
#define MCL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"
#include "MCL.h"
#include "map_utils.h"
#include "robot_utils.h"

class MCLNode : public rclcpp::Node {
public:
    MCLNode();
    ~MCLNode();

private:
    void initializeParameters();
    void initializePublishersAndSubscribers();
    void initializeMessages();
    void initializeMCLComponents();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void calcControl();
    void update();
    geometry_msgs::msg::PoseWithCovariance calculateMeanPose() const;

    // Node components
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_pub_, local_map_pc_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mcl_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Messages
    nav_msgs::msg::OccupancyGrid grid_map_;
    nav_msgs::msg::Odometry mcl_msg_;
    sensor_msgs::msg::PointCloud2 map_pc_msg_, local_map_pc_msg_;

    // State tracking
    geometry_msgs::msg::Pose last_pose_, pose_;
    Robot robot_;
    command control_;

    // MCL parameters
    int particles_N_;
    double pc_ang_sep_;
    int update_rate_;

    // CUDA memory
    particle* particles_;
    particle* belief_;
    double* z_;
    Point* local_map_tmp_;
    size_t particles_size_;

    // Map data
    size_t map_size_;
    int map_N_;
    Point* map_;

    // Local map data
    size_t local_map_size_;
    int local_map_N_;
    Point* local_map_;
};

#endif // MCL_NODE_H