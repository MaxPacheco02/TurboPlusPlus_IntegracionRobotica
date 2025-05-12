#ifndef MCL_NODE_H
#define MCL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"
#include "map_utils.h"
// #include "robot_utils.h"
#include "MCL_test.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <random>

class MCLNode : public rclcpp::Node
{
public:
    MCLNode();
    ~MCLNode();

private:
    void initializeParameters();
    void initializePublishersAndSubscribers();
    void initializeMessages();
    void initializeMCLComponents(double init_x, double init_y, double init_theta);
    void initializeEmptyMap();
    void updateCell(int x, int y, int value);
    bool worldToGrid(double wx, double wy, int &gx, int &gy);
    void gridToWorld(int gx, int gy, double &wx, double &wy);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void estimateCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void calcControl();
    void update();
    void publish_msgs();

    void initializeMap();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void updateMapFromLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void updateGridCell(double robot_x, double robot_y, double point_x, double point_y);
    void updatePointCloudMap(const std::vector<Point> &new_points);

    void expandMapIfNeeded(
        nav_msgs::msg::OccupancyGrid &map_msg,
        const geometry_msgs::msg::Pose &robot_pose,
        double expansion_margin = 2.0);

    void updateMapFromRaycast(
        nav_msgs::msg::OccupancyGrid &map_msg,
        const geometry_msgs::msg::Pose &robot_pose,
        const std::vector<Point> &ray_endpoints,
        double max_range = 10.0);

    geometry_msgs::msg::PoseWithCovariance calculateMeanPose() const;

    // Node components
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_pub_, local_map_pc_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mcl_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_estimate_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Messages
    nav_msgs::msg::OccupancyGrid grid_map_;
    nav_msgs::msg::OccupancyGrid map_msg_;
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

    // Particle data (replacing CUDA memory)
    std::vector<particle> particles_;
    std::vector<particle> belief_;
    std::vector<double> z_;

    // Map data
    int map_N_;
    int max_map_N_{50000};
    std::vector<Point> map_;

    // Local map data
    int local_map_N_;
    std::vector<Point> local_map_;
    std::vector<Point> local_map_tmp_;

    // Laser data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    std::vector<double> filtered_ranges_;
    double laser_min_angle_;
    double laser_angle_increment_;

    // Random generator for particle initialization
    std::random_device rd_;
    std::mt19937 gen_;
};

#endif // MCL_NODE_H
