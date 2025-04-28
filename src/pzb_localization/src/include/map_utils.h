#ifndef MAP_UTILS_H
#define MAP_UTILS_H

#include <geometry_msgs/msg/pose.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

#include <cuda_runtime.h>

// Basic 2D point structure
struct Point {
    double x;
    double y;
};

// Structure for point cloud differencing
struct PCDiff {
    Point p;
    double dist;
    double ang;
};

/**
 * Point cloud message operations
 */
void resize_pc(
    sensor_msgs::msg::PointCloud2& pc_msg,
    int width);

void init_pc_msg(
    sensor_msgs::msg::PointCloud2& pc_msg);

uint32_t packRGBA(
    uint8_t r, uint8_t g, uint8_t b, uint8_t a);

void fill_pc(
    sensor_msgs::msg::PointCloud2& pc_msg,
    const Point* map_arr, int map_size,
    uint8_t r, uint8_t g, uint8_t b, uint8_t a,
    double z);

/**
 * Map conversion operations
 */
int map2arr(
    const nav_msgs::msg::OccupancyGrid& map,
    Point* points_array,
    int max_points);

/**
 * Raycast operations
 */
__host__ __device__ void compute_raycast(
    const Point* map, int map_N,
    Point* local_map,
    const geometry_msgs::msg::Pose& source_pose,
    double angle_separation);

#endif // MAP_UTILS_H