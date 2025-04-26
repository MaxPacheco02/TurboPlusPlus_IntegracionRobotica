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

// #include "include/map_utils.h"
// #include "include/robot.h"
#include "include/map_utils.h"
#include "include/robot.h"
#include "include/robot_utils.h"
#include "include/MCL.h"
#include "include/utility.h"
#include <cuda_runtime.h>
#include "cuda_kernels.h"
// #include "include/utility.h"


using namespace std::chrono_literals;

class MCLNode : public rclcpp::Node
{
public:
    MCLNode() : Node("mcl_node")
    {
        using namespace std::placeholders;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pzb/odom", 10,
            [this](const nav_msgs::msg::Odometry &msg)
            {
                pose = msg.pose.pose;
            });

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/pzb/map", 10,
            [this](const nav_msgs::msg::OccupancyGrid &msg)
            {
                if (grid_map.data.size() == msg.data.size())
                    return;

                grid_map = msg;

                int max_map_size = msg.data.size() / 10;
                map_size = max_map_size * sizeof(Point);
                checkCuda(cudaMallocManaged(&map, map_size));

                map_N = map2arr(grid_map, map, max_map_size);

                fill_pc(map_pc_msg, map, map_N, 0, 0, 255, 255, 0.0);
            });

        map_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pzb/map_pc", 10);
        local_map_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pzb/pc", 10);
        mcl_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mcl/odom", 10);

        timer_ = this->create_wall_timer(10ms, std::bind(&MCLNode::update, this));

        /*
            ====================
            Initialize ROS2 msgs
            ====================
        */
        init_pc_msg(map_pc_msg);
        init_pc_msg(local_map_pc_msg);

        mcl_msg.header.frame_id = "odom";

        pose.position = geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0);
        pose.orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1);

        /*
            ========================
            Initialize MCL variables
            ========================
        */

        // Setup robot (MCL) variables
        robot.set_states(0.0, 0.0, 0.0);

        // robot.set_noise(0.05, 0.05, 0.1);   // Good performance
        robot.set_noise(0.05, 0.01, 0.1);

        // Initialize particle array
        particles_size = particles_N * sizeof(particle);
        checkCuda(cudaMallocManaged(&particles, particles_size));
        particle particle_tmp{robot, 0.0};

        // Fill array with empty poses
        for (int i = 0; i < particles_N; i++)
        {
            particles[i] = particle_tmp;
        }

        // Initialize local map array
        local_map_N = static_cast<int>(360.0 / pc_ang_sep);
        local_map_size = local_map_N * sizeof(Point);
        checkCuda(cudaMallocManaged(&local_map, local_map_size));
    }

    void update()
    {
        mcl();
        
        // The rest of your function
        map_pc_msg.header.stamp = this->get_clock()->now();
        local_map_pc_msg.header.stamp = this->get_clock()->now();
        mcl_msg.header.stamp = this->get_clock()->now();
    
        map_pc_pub_->publish(map_pc_msg);
        local_map_pc_pub_->publish(local_map_pc_msg);
    
        mcl_pub_->publish(mcl_msg);
    }

    void mcl()
    {

        // Update raycast pc2 msg
        compute_raycast(map, map_N, local_map, pose, pc_ang_sep);
        fill_pc(local_map_pc_msg, local_map, local_map_N, 255, 0, 0, 255, 0.05);

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
        if (control.turn > M_PI)
            control.turn -= 2 * M_PI;
        if (control.turn < -M_PI)
            control.turn += 2 * M_PI;

        // Execute movement (with noise)
        // We don't care, it's just for the GUI
        // robot.move(control.turn, control.forward);

        // Read sensor
        double *z;
        size_t sensor_size = local_map_N * sizeof(double);
        checkCuda(cudaMallocManaged(&z, sensor_size));

        Point* local_map_tmp;
        checkCuda(cudaMallocManaged(&local_map_tmp, local_map_size));
    
        // IRL: Real LiDAR's data
        robot.sense(map, map_N, local_map_tmp, local_map_N, z, pc_ang_sep, pose, true);

        // Run MCL
        particle *belief;
        checkCuda(cudaMallocManaged(&belief, particles_size));

        MCL(particles, particles_N, control, z, local_map_N, map, map_N, pc_ang_sep).resample(belief);    

        // Visualize result
        // utility::visualization(&robot, 0, &particles, &belief, map);

        // Update particles
        cp_particle_from_to(belief, particles, particles_N);

        mcl_msg.pose = get_pondered_pose();

        last_pose = pose;

        // Free malloc
        checkCuda(cudaFree(belief));
        checkCuda(cudaFree(local_map_tmp));
        checkCuda(cudaFree(z));
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

    Robot robot;
    command control;
    particle *particles;
    int particles_N = 500;
    size_t particles_size;

    double pc_ang_sep = 9.0;

    // CUDA vars
    size_t map_size;
    int map_N;
    Point *map;

    size_t local_map_size;
    int local_map_N;
    Point *local_map;

    geometry_msgs::msg::PoseWithCovariance get_pondered_pose()
    {
        geometry_msgs::msg::PoseWithCovariance pose_with_cov;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Quaternion orientation;

        if (particles_N == 0)
        {
            return pose_with_cov;
        }

        // Calculate weighted mean position and orientation
        double total_weight = 0.0;
        double x_weighted_sum = 0.0;
        double y_weighted_sum = 0.0;
        double cos_weighted_sum = 0.0;
        double sin_weighted_sum = 0.0;

        for (int i = 0; i < particles_N; i++)
        {
            double x = particles[i].r.get_x();
            double y = particles[i].r.get_y();
            double theta = particles[i].r.get_theta();
            double weight = particles[i].w;

            x_weighted_sum += x * weight;
            y_weighted_sum += y * weight;
            cos_weighted_sum += std::cos(theta) * weight;
            sin_weighted_sum += std::sin(theta) * weight;

            total_weight += weight;
        }

        // Handle case where weights sum to zero
        if (total_weight <= 0.0)
        {
            // Fallback: use the first particle
            position.x = particles[0].r.get_x();
            position.y = particles[0].r.get_y();

            tf2::Quaternion q;
            q.setRPY(0, 0, particles[0].r.get_theta());
            tf2::convert(q, orientation);

            pose_with_cov.pose = geometry_msgs::build<geometry_msgs::msg::Pose>().position(position).orientation(orientation);
            return pose_with_cov;
        }

        // Calculate mean values
        double x_mean = x_weighted_sum / total_weight;
        double y_mean = y_weighted_sum / total_weight;
        double theta_mean = std::atan2(
            sin_weighted_sum / total_weight,
            cos_weighted_sum / total_weight);

        // Set the mean pose
        position.x = x_mean;
        position.y = y_mean;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_mean);
        tf2::convert(q, orientation);

        pose_with_cov.pose = geometry_msgs::build<geometry_msgs::msg::Pose>().position(position).orientation(orientation);

        // Calculate covariance matrix
        // Initialize covariance matrix (6x6 for x, y, z, roll, pitch, yaw)
        std::array<double, 36> cov;
        std::fill(cov.begin(), cov.end(), 0.0);

        for (int i = 0 ; i < particles_N ; i++)
        {
            double x = particles[i].r.get_x();
            double y = particles[i].r.get_y();
            double theta = particles[i].r.get_theta();
            double weight = particles[i].w / total_weight; // Normalize weight

            // Handle angle difference properly (considering circular nature)
            double theta_diff = theta - theta_mean;
            if (theta_diff > M_PI)
                theta_diff -= 2 * M_PI;
            if (theta_diff < -M_PI)
                theta_diff += 2 * M_PI;

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
        cov[0] = std::max(cov[0], min_variance);   // x variance
        cov[7] = std::max(cov[7], min_variance);   // y variance
        cov[14] = min_variance;                    // z variance (unused)
        cov[21] = min_variance;                    // roll variance (unused)
        cov[28] = min_variance;                    // pitch variance (unused)
        cov[35] = std::max(cov[35], min_variance); // yaw variance

        // Set covariance
        for (size_t i = 0; i < 36; ++i)
        {
            pose_with_cov.covariance[i] = cov[i];
        }

        return pose_with_cov;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MCLNode>());
    rclcpp::shutdown();
    return 0;
}
