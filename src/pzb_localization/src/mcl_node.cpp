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
            
        map_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pzb/map_pc", 10); // Full map with pointclouds???
        local_map_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pzb/pc", 10); // Laser scan from lidar
        mcl_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mcl/odom", 10);

        timer_=this->create_wall_timer(10ms,std::bind(&MCLNode::update,this));

        // Initialize PointCloud msgs
        init_pc_msg(map_pc_msg);
        init_pc_msg(local_map_pc_msg);

        // Initialize MCL Odometry msg
        mcl_msg.header.frame_id = "odom";

        // Setup robot (MCL) variables
        robot.set_states(0.0, 0.0, 0.0);

        // robot.set_noise(0.05, 0.05, 0.1);   // Good performance
        robot.set_noise(0.05, 0.01, 0.1);

        for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
        {
            particles.emplace_back(robot, 0.0);
        }

        pose.position = geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0);
        pose.orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1);

    }

    void update(){
        mcl();

        map_pc_msg.header.stamp = this->get_clock()->now();
        local_map_pc_msg.header.stamp = this->get_clock()->now();
        mcl_msg.header.stamp = this->get_clock()->now();

        map_pc_pub_->publish(map_pc_msg);
        local_map_pc_pub_->publish(local_map_pc_msg);

        mcl_pub_->publish(mcl_msg);

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
        // utility::visualization(&robot, 0, &particles, &belief, map);

        // Update particles
        particles = belief;

        mcl_msg.pose = get_pondered_pose(belief);

        last_pose = pose;
    }
    
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_pub_, local_map_pc_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mcl_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::Pose last_pose, pose;

    nav_msgs::msg::OccupancyGrid grid_map;
    nav_msgs::msg::Odometry mcl_msg;
    
    sensor_msgs::msg::PointCloud2 map_pc_msg;
    sensor_msgs::msg::PointCloud2 local_map_pc_msg;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Point> map, local_map;

    Robot robot;
    command control;
    std::vector<particle> particles;
    int NUMBER_OF_PARTICLES = 500;

    geometry_msgs::msg::PoseWithCovariance get_pondered_pose(
        const std::vector<particle>& particles
    ) {
        geometry_msgs::msg::PoseWithCovariance pose_with_cov;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Quaternion orientation;
        
        if (particles.empty()) {
            return pose_with_cov;
        }
        
        // Calculate weighted mean position and orientation
        double total_weight = 0.0;
        double x_weighted_sum = 0.0;
        double y_weighted_sum = 0.0;
        double cos_weighted_sum = 0.0;
        double sin_weighted_sum = 0.0;
        
        for (const auto& p : particles) {
            double x = p.r.get_x();
            double y = p.r.get_y();
            double theta = p.r.get_theta();
            double weight = p.w;
            
            x_weighted_sum += x * weight;
            y_weighted_sum += y * weight;
            cos_weighted_sum += std::cos(theta) * weight;
            sin_weighted_sum += std::sin(theta) * weight;
            
            total_weight += weight;
        }
        
        // Handle case where weights sum to zero
        if (total_weight <= 0.0) {
            if (!particles.empty()) {
                // Fallback: use the first particle
                position.x = particles[0].r.get_x();
                position.y = particles[0].r.get_y();
                
                tf2::Quaternion q;
                q.setRPY(0, 0, particles[0].r.get_theta());
                tf2::convert(q, orientation);
            }
            pose_with_cov.pose = geometry_msgs::build
                <geometry_msgs::msg::Pose>().
                position(position).orientation(orientation);
            return pose_with_cov;
        }
        
        // Calculate mean values
        double x_mean = x_weighted_sum / total_weight;
        double y_mean = y_weighted_sum / total_weight;
        double theta_mean = std::atan2(
            sin_weighted_sum / total_weight, 
            cos_weighted_sum / total_weight
        );
        
        // Set the mean pose
        position.x = x_mean;
        position.y = y_mean;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_mean);
        tf2::convert(q, orientation);

        pose_with_cov.pose = geometry_msgs::build
        <geometry_msgs::msg::Pose>().
        position(position).orientation(orientation);


        
        // Calculate covariance matrix
        // Initialize covariance matrix (6x6 for x, y, z, roll, pitch, yaw)
        std::array<double, 36> cov;
        std::fill(cov.begin(), cov.end(), 0.0);
        
        for (const auto& p : particles) {
            double x = p.r.get_x();
            double y = p.r.get_y();
            double theta = p.r.get_theta();
            double weight = p.w / total_weight; // Normalize weight
            
            // Handle angle difference properly (considering circular nature)
            double theta_diff = theta - theta_mean;
            if (theta_diff > M_PI) theta_diff -= 2 * M_PI;
            if (theta_diff < -M_PI) theta_diff += 2 * M_PI;
            
            // Update covariance components (only for x, y, and yaw)
            // Format is row-major [x, y, z, roll, pitch, yaw]
            // x variance (index 0)
            cov[0] += weight * (x - x_mean) * (x - x_mean);
            // y-x covariance (index 1) and x-y covariance (index 6)
            cov[1] += weight * (x - x_mean) * (y - y_mean);
            cov[6] = cov[1]; // Symmetric matrix
            // y variance (index 7)
            cov[7] += weight * (y - y_mean) * (y - y_mean);
            // x-yaw covariance (index 5) and yaw-x covariance (index 30)
            cov[5] += weight * (x - x_mean) * theta_diff;
            cov[30] = cov[5]; // Symmetric matrix
            // y-yaw covariance (index 11) and yaw-y covariance (index 31)
            cov[11] += weight * (y - y_mean) * theta_diff;
            cov[31] = cov[11]; // Symmetric matrix
            // yaw variance (index 35)
            cov[35] += weight * theta_diff * theta_diff;
        }
        
        // Add small values to diagonal to ensure positive definiteness
        const double min_variance = 1e-6;
        cov[0] = std::max(cov[0], min_variance);  // x variance
        cov[7] = std::max(cov[7], min_variance);  // y variance
        cov[14] = min_variance; // z variance (unused)
        cov[21] = min_variance; // roll variance (unused)
        cov[28] = min_variance; // pitch variance (unused)
        cov[35] = std::max(cov[35], min_variance); // yaw variance

        // Set covariance
        for (size_t i = 0; i < 36; ++i) {
            pose_with_cov.covariance[i] = cov[i];
        }
        
        return pose_with_cov;
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MCLNode>());
    rclcpp::shutdown();
    return 0;
}
