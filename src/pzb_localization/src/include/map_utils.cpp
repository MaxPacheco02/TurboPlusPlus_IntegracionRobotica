#include "map_utils.h"

#include <iostream>
#include <math.h>

// Resize a pc2 msg's data array
void resize_pc(sensor_msgs::msg::PointCloud2& pc_msg, int width)
{
    pc_msg.width = width;
    pc_msg.row_step = pc_msg.point_step * pc_msg.width;

    // Allocate memory for the point data
    pc_msg.data.resize(pc_msg.row_step * pc_msg.height);
}

// Initialize a pc2 msg
void init_pc_msg(sensor_msgs::msg::PointCloud2& pc_msg)
{
    pc_msg.header.frame_id = "odom";

    pc_msg.height = 1;
    pc_msg.width = 4; // Number of pointclouds

    std::array<std::string, 4> field_names{"x", "y", "z", "rgba"};
    int float32_size = 4;

    // Fill xyz fields
    for(int i = 0 ; i < field_names.size() ; i++){
        pc_msg.fields.push_back(
            sensor_msgs::build<sensor_msgs::msg::PointField>()
            .name(field_names[i])
            .offset(i*float32_size)
            .datatype(sensor_msgs::msg::PointField::FLOAT32)
            .count(1)
        );
    }

    // Correct rgba field's datatype
    pc_msg.fields[3].datatype = 
        sensor_msgs::msg::PointField::UINT32;

    // # of fields * 4 bytes (float32)
    pc_msg.point_step = 
        pc_msg.fields.size() * float32_size;
    
    pc_msg.is_dense = true;

    resize_pc(pc_msg, pc_msg.width);
}

// Method to pack R,G,B,A values into a single uint32_t
uint32_t packRGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    return ((uint32_t)b << 0) | 
            ((uint32_t)g << 8) | 
            ((uint32_t)r << 16) | 
            ((uint32_t)a << 24);
}

// Method to fill a pc2 msg from an array containing data coordinates
void fill_pc(sensor_msgs::msg::PointCloud2& pc_msg,
    const std::vector<Point>& pc_data, uint8_t r, uint8_t g, uint8_t b, uint8_t a, double z) 
{
    resize_pc(pc_msg, pc_data.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgba(pc_msg, "rgba");

    uint32_t color = packRGBA(r,g,b,a);

    for(int i = 0 ; i < pc_data.size() ; i++){
        *iter_x = pc_data[i].x;
        *iter_y = pc_data[i].y;
        *iter_z = z;
        *iter_rgba = color;
        
        // Move to next point
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_rgba;
    }
}

std::vector<Point> map2vec(const nav_msgs::msg::OccupancyGrid& map)
{
    std::vector<Point> points;

    if(map.data.empty()){
        return points;
    }

    // We're estimating 10% of the map is boundary
    points.reserve(map.data.size() / 10);

    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;
    const double resolution = map.info.resolution;
    const int width = map.info.width;
    const int height = map.info.height;

    const int dx[4] = {1, 0, -1, 0};
    const int dy[4] = {0, 1, 0, -1};
    const int neighbor_offsets[4] = {1, width, -1, -width};

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            
            // Skip non-occupied cells
            if (map.data[idx] < 100 && map.data[idx] >= 0) {
                continue;
            }
            
            // Check neighbors more efficiently
            for (int n = 0; n < 4; n++) {
                int nx = x + dx[n];
                int ny = y + dy[n];
                
                // Skip if outside map
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    continue;
                }
                
                int neighbor_idx = idx + neighbor_offsets[n];
                
                // If neighbor is free space, this is a boundary
                if (map.data[neighbor_idx] == 0) {
                    // Convert to world coordinates
                    double world_x = origin_x + (x + 0.5) * resolution;
                    double world_y = origin_y + (y + 0.5) * resolution;
                    
                    points.push_back({world_x, world_y});
                    break;  // Once we know it's a boundary, no need to check other neighbors
                }
            }
        }
    }

    return points;
}

void compute_raycast(
    const std::vector<Point>& map, std::vector<Point>& visible_points,
    const geometry_msgs::msg::Pose& source_pose, double angle_separation
){
    Point source{source_pose.position.x, source_pose.position.y};

    // Convert map points into polar coordinates (distance, angle)
    std::vector<PCDiff> pc_data;
    pc_data.reserve(map.size());

    for (const auto& pt : map)
    {
        double dx = pt.x - source.x;
        double dy = pt.y - source.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        double ang = std::atan2(dy, dx);
        if (ang < 0) ang += 2.0 * M_PI;

        pc_data.push_back(PCDiff{pt, dist, ang});
    }

    // Angular resolution and number of rays
    const double angle_resolution = angle_separation * M_PI / 180.0;
    const int num_rays = static_cast<int>(2.0 * M_PI / angle_resolution) + 1;

    // Bin points by angle
    std::vector<std::vector<PCDiff>> angle_bins(num_rays);
    for (const auto& p : pc_data)
    {
        int bin_idx = static_cast<int>(p.ang / angle_resolution);
        if (bin_idx >= num_rays) bin_idx = num_rays - 1;
        angle_bins[bin_idx].push_back(p);
    }

    // Clear and reserve space for output
    visible_points.clear();
    visible_points.reserve(num_rays);

    // Wide angular threshold
    const double angle_threshold = 0.3 * angle_resolution;

    for (int i = 0; i < num_rays; ++i)
    {
        double ray_angle = i * angle_resolution;
        double closest_dist = std::numeric_limits<double>::max();
        Point closest_point{0.0, 0.0};
        bool found = false;

        // Search neighboring bins for better coverage (angle wrap-safe)
        for (int offset = -2; offset <= 2; ++offset)
        {
            int neighbor_idx = (i + offset + num_rays) % num_rays;
            for (const auto& p : angle_bins[neighbor_idx])
            {
                double diff = std::abs(p.ang - ray_angle);
                if (diff > M_PI) diff = 2.0 * M_PI - diff;

                if (diff <= angle_threshold && p.dist < closest_dist)
                {
                    closest_dist = p.dist;
                    closest_point = p.p;
                    found = true;
                }
            }
        }

        visible_points.push_back(closest_point);
    }
}