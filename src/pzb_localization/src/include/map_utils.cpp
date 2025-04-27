#include "map_utils.h"
#include <cmath>
#include <algorithm>

void resize_pc(sensor_msgs::msg::PointCloud2& pc_msg, int width) {
    pc_msg.width = width;
    pc_msg.row_step = pc_msg.point_step * width;
    pc_msg.data.resize(pc_msg.row_step * pc_msg.height);
}

void init_pc_msg(sensor_msgs::msg::PointCloud2& pc_msg) {
    // Set basic properties
    pc_msg.header.frame_id = "odom";
    pc_msg.height = 1;
    pc_msg.width = 4;

    // Define field names and properties
    const std::array<std::string, 4> field_names{"x", "y", "z", "rgba"};
    constexpr int float32_size = 4;

    // Add fields
    pc_msg.fields.clear();
    for (size_t i = 0; i < field_names.size(); i++) {
        sensor_msgs::msg::PointField field;
        field.name = field_names[i];
        field.offset = i * float32_size;
        field.datatype = (i == 3) ? 
            sensor_msgs::msg::PointField::UINT32 :
            sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        pc_msg.fields.push_back(field);
    }

    pc_msg.point_step = field_names.size() * float32_size;
    pc_msg.is_dense = true;

    resize_pc(pc_msg, pc_msg.width);
}

uint32_t packRGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    return ((uint32_t)r << 16) |
           ((uint32_t)g << 8) |
           ((uint32_t)b << 0) |
           ((uint32_t)a << 24);
}

void fill_pc(
    sensor_msgs::msg::PointCloud2& pc_msg,
    const Point* map_arr, int map_size,
    uint8_t r, uint8_t g, uint8_t b, uint8_t a,
    double z) 
{
    resize_pc(pc_msg, map_size);

    sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgba(pc_msg, "rgba");

    const uint32_t color = packRGBA(r, g, b, a);

    for (int i = 0; i < map_size; i++) {
        *iter_x = map_arr[i].x;
        *iter_y = map_arr[i].y;
        *iter_z = z;
        *iter_rgba = color;

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_rgba;
    }
}

int map2arr(
    const nav_msgs::msg::OccupancyGrid& map,
    Point* points_array,
    int max_points) 
{
    if (map.data.empty()) return 0;

    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;
    const double resolution = map.info.resolution;
    const int width = map.info.width;
    const int height = map.info.height;

    // Neighbor offsets for boundary detection
    const std::array<int, 4> dx = {1, 0, -1, 0};
    const std::array<int, 4> dy = {0, 1, 0, -1};
    const std::array<int, 4> neighbor_offsets = {
        1, width, -1, -width
    };

    int count = 0;

    // Scan through map cells
    for (int y = 0; y < height && count < max_points; y++) {
        for (int x = 0; x < width && count < max_points; x++) {
            const int idx = y * width + x;

            // Skip non-occupied cells
            if (map.data[idx] < 100 && map.data[idx] >= 0) continue;

            // Check neighbors for boundary
            bool is_boundary = false;
            for (int n = 0; n < 4; n++) {
                const int nx = x + dx[n];
                const int ny = y + dy[n];

                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    continue;
                }

                const int neighbor_idx = idx + neighbor_offsets[n];
                if (map.data[neighbor_idx] == 0) {
                    is_boundary = true;
                    break;
                }
            }

            if (is_boundary) {
                // Convert to world coordinates
                points_array[count].x = origin_x + (x + 0.5) * resolution;
                points_array[count].y = origin_y + (y + 0.5) * resolution;
                count++;
            }
        }
    }

    return count;
}

__device__ __host__ void compute_raycast(
    const Point* map, int map_N,
    Point* local_map,
    const geometry_msgs::msg::Pose& source_pose,
    double angle_separation)
{
    const Point source{
        source_pose.position.x,
        source_pose.position.y
    };

    // Convert to radians and compute constants
    const double angle_resolution = angle_separation * M_PI / 180.0;
    const int num_rays = static_cast<int>(2.0 * M_PI / angle_resolution) + 1;
    const double angle_threshold = 0.1 * angle_resolution;

    // Process each ray
    for (int i = 0; i < num_rays; ++i) {
        const double ray_angle = i * angle_resolution;
        double closest_dist = INFINITY;
        Point closest_point{0.0, 0.0};

        // Find closest point along this ray
        for (int j = 0; j < map_N; ++j) {
            const double dx = map[j].x - source.x;
            const double dy = map[j].y - source.y;
            const double dist = sqrt(dx * dx + dy * dy);
            double point_angle = atan2(dy, dx);

            // Normalize angle
            if (point_angle < 0) {
                point_angle += 2.0 * M_PI;
            }

            // Compute angle difference
            double diff = fabs(point_angle - ray_angle);
            if (diff > M_PI) {
                diff = 2.0 * M_PI - diff;
            }

            // Update if this is the closest point so far
            if (diff < angle_threshold && dist < closest_dist) {
                closest_dist = dist;
                closest_point = map[j];
            }
        }

        local_map[i] = closest_point;
    }
}