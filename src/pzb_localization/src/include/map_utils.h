#ifndef MAP_UTILS_H
#define MAP_UTILS_H

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <limits>

struct Point{
    double x;
    double y;
};

struct PCDiff{
    Point p;
    double dist;
    double ang;
};

void resize_pc(
    sensor_msgs::msg::PointCloud2& pc_msg,
    int width
);

void init_pc_msg(
    sensor_msgs::msg::PointCloud2& pc_msg
);

uint32_t packRGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a);

void fill_pc(
    sensor_msgs::msg::PointCloud2& pc_msg,
    const std::vector<Point>& pc_data, 
    uint8_t r, uint8_t g, uint8_t b, uint8_t a,
    double z
);

std::vector<Point> map2vec(
    const nav_msgs::msg::OccupancyGrid& map
);

std::vector<Point> map2vector_cpu(
  const nav_msgs::msg::OccupancyGrid& map, int max_points = -1
);

void compute_raycast(
    const std::vector<Point>& map, std::vector<Point>& visible_points,
    const geometry_msgs::msg::Pose& source_pose, double angle_separation
);

void compute_raycast_cpu(
    const std::vector<Point>& map,       // input map
    std::vector<Point>& local_map,       // output map (preallocated)
    const geometry_msgs::msg::Pose& source_pose,
    double angle_separation_deg
);

#endif
