#include "mcl_node.h"
#include <chrono>

using namespace std::chrono_literals;

MCLNode::MCLNode() : Node("mcl_node")
{
    // Initialize map-related variables
    map_ = nullptr;
    map_N_ = 0;

    // Rest of your initialization code...
    initializeParameters();
    initializePublishersAndSubscribers();
    initializeMessages();
    initializeMCLComponents(0.0, 0.0, 0.0);
}

MCLNode::~MCLNode()
{
    if (map_ != nullptr)
        cudaFree(map_);
    if (local_map_ != nullptr)
        cudaFree(local_map_);
    if (particles_ != nullptr)
        cudaFree(particles_);
    if (belief_ != nullptr)
        cudaFree(belief_);
    if (z_ != nullptr)
        cudaFree(z_);
    if (local_map_tmp_ != nullptr)
        cudaFree(local_map_tmp_);
}

void MCLNode::initializeMap()
{
    // Initialize empty map with default size
    map_msg_.info.resolution = 0.1;                      // cm per cell
    map_msg_.info.width = 10 / map_msg_.info.resolution;  // 10 meters
    map_msg_.info.height = 10 / map_msg_.info.resolution; // 10 meters

    // Center the map at robot's initial position
    map_msg_.info.origin.position.x = -5.0; // -5m to +5m range
    map_msg_.info.origin.position.y = -5.0;
    map_msg_.info.origin.position.z = 0.0;

    // Initialize all cells as unknown (-1)
    map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height, -1);

    // Set header frame
    map_msg_.header.frame_id = "odom";

    // Initialize point cloud map
    map_size_ = max_map_N_ * sizeof(Point);
    if (map_ != nullptr)
    {
        cudaFree(map_);
    }
    checkCuda(cudaMallocManaged(&map_, map_size_));
    map_N_ = 0;
}

void MCLNode::initializeParameters()
{
    particles_N_ = 500;
    pc_ang_sep_ = 10.0; // Reduced from 9.0 for better angular resolution
    update_rate_ = 50;  // Hz
}

void MCLNode::initializePublishersAndSubscribers()
{
    // Create publishers
    map_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "pzb/map_pc", 10);
    local_map_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "pzb/pc", 10);
    mcl_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "mcl/odom", 10);

    // Matching QoS for map_server node
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos_profile);
    // map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    // Create subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "pzb/odom", 10,
        std::bind(&MCLNode::odomCallback, this, std::placeholders::_1));

    initial_estimate_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10,
        std::bind(&MCLNode::estimateCallback, this, std::placeholders::_1));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "pzb/map", 10,
        std::bind(&MCLNode::mapCallback, this, std::placeholders::_1));

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(&MCLNode::laserCallback, this, std::placeholders::_1));

    // Create timer
    timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / update_rate_),
        std::bind(&MCLNode::update, this));
}

void MCLNode::initializeMessages()
{
    init_pc_msg(map_pc_msg_);
    init_pc_msg(local_map_pc_msg_);
    mcl_msg_.header.frame_id = "odom";

    // Initialize poses
    pose_.position = geometry_msgs::build<geometry_msgs::msg::Point>()
                         .x(0.0)
                         .y(0.0)
                         .z(0.0);
    pose_.orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                            .x(0.0)
                            .y(0.0)
                            .z(0.0)
                            .w(1.0);
    last_pose_ = pose_;
}

void MCLNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Initialize map if it hasn't been done yet
    if (map_ == nullptr || map_N_ == 0)
    {
        initializeMap();
    }

    // Store laser parameters
    laser_min_angle_ = msg->angle_min;
    laser_angle_increment_ = msg->angle_increment;

    // Filter ranges according to pc_ang_sep_
    filtered_ranges_.clear();

    // With 720 points (0.5 degrees of separation), the i is 1
    // For 360 points (1 deg of sep), the i would be 2...
    // For 180 points (2 degs of sep), the i would be 4
    // For 72 points (5 degrees of sep), the i would be 10

    // n_desired = 360 / ang_sep
    // n_desired * skip_i = msg->ranges.size()

    // skip_i = msg->ranges.size() / (360 / ang_sep)

    // int skip_rays = static_cast<int>((pc_ang_sep_ * M_PI / 180.0) / msg->angle_increment);
    int skip_rays = msg->ranges.size() / (360 / pc_ang_sep_);
    skip_rays = std::max(1, skip_rays);
    
    RCLCPP_ERROR(get_logger(), "Original scan size: %d.", msg->ranges.size());
    RCLCPP_ERROR(get_logger(), "Estimated original scan size: %f.", 2*M_PI / msg->angle_increment);
    RCLCPP_ERROR(get_logger(), "Proposed index jump from original scan: %d.", skip_rays);
    RCLCPP_ERROR(get_logger(), "Which would generate %f points", 2*M_PI / (skip_rays*msg->angle_increment));

    for (size_t i = 0; i < msg->ranges.size(); i += skip_rays)
    {
        if (std::isfinite(msg->ranges[i]) &&
            msg->ranges[i] >= msg->range_min &&
            msg->ranges[i] <= msg->range_max)
        {
            filtered_ranges_.push_back(msg->ranges[i]);
        }
    }

    // Update map and point cloud
    updateMapFromLaser(msg);
}

void MCLNode::initializeEmptyMap()
{
    // Set basic map properties
    map_msg_.info.resolution = 0.05; // 5cm per cell
    map_msg_.info.width = 200;       // 10 meters / 0.05
    map_msg_.info.height = 200;      // 10 meters / 0.05

    // Set map origin (bottom-left corner)
    map_msg_.info.origin.position.x = -5.0; // Center map at robot
    map_msg_.info.origin.position.y = -5.0;
    map_msg_.info.origin.position.z = 0.0;

    // Initialize all cells as unknown (-1)
    map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height, -1);

    // Set header frame
    map_msg_.header.frame_id = "odom";
}

void MCLNode::updateCell(int x, int y, int value)
{
    if (x >= 0 && x < map_msg_.info.width &&
        y >= 0 && y < map_msg_.info.height)
    {
        map_msg_.data[y * map_msg_.info.width + x] = value;
    }
}

// Convert world coordinates to grid coordinates
bool MCLNode::worldToGrid(double wx, double wy, int &gx, int &gy)
{
    gx = static_cast<int>((wx - map_msg_.info.origin.position.x) / map_msg_.info.resolution);
    gy = static_cast<int>((wy - map_msg_.info.origin.position.y) / map_msg_.info.resolution);

    return (gx >= 0 && gx < map_msg_.info.width &&
            gy >= 0 && gy < map_msg_.info.height);
}

// Convert grid coordinates to world coordinates
void MCLNode::gridToWorld(int gx, int gy, double &wx, double &wy)
{
    wx = gx * map_msg_.info.resolution + map_msg_.info.origin.position.x;
    wy = gy * map_msg_.info.resolution + map_msg_.info.origin.position.y;
}

void MCLNode::initializeMCLComponents(double init_x, double init_y, double init_theta)
{
    // Initialize robot
    robot_.set_states(0.0, 0.0, 0.0);
    robot_.set_noise(0.02, 0.05, 0.02);

    // Initialize CUDA memory
    particles_size_ = particles_N_ * sizeof(particle);
    local_map_N_ = static_cast<int>(360.0 / pc_ang_sep_);
    local_map_size_ = local_map_N_ * sizeof(Point);

    // Allocate memory
    checkCuda(cudaMallocManaged(&particles_, particles_size_));
    checkCuda(cudaMallocManaged(&local_map_, local_map_size_));
    checkCuda(cudaMallocManaged(&belief_, particles_size_));
    checkCuda(cudaMallocManaged(&z_, local_map_N_ * sizeof(double)));
    checkCuda(cudaMallocManaged(&local_map_tmp_, local_map_size_));

    // Improved initialization strategy: multi-modal distribution
    const double init_pos_var = 0.1;    // 10cm variance in position
    const double init_angle_var = 0.05; // ~2.8 degrees variance in orientation

    std::random_device rd;
    std::mt19937 gen(rd());

    // Initialize particles with multiple strategies
    int particles_per_strategy = particles_N_ / 3;

    // Strategy 1: Tight normal distribution around initial pose (33%)
    std::normal_distribution<double> tight_pos_dist(0.0, init_pos_var * 0.5);
    std::normal_distribution<double> tight_angle_dist(0.0, init_angle_var * 0.5);

    for (int i = 0; i < particles_per_strategy; i++)
    {
        Robot r;
        r.set_states(
            init_x + tight_pos_dist(gen),
            init_y + tight_pos_dist(gen),
            init_theta + tight_angle_dist(gen));
        r.set_noise(0.02, 0.05, 0.02);
        particles_[i] = particle{r, 1.0 / particles_N_};
    }

    // Strategy 2: Wider normal distribution (33%)
    std::normal_distribution<double> wide_pos_dist(0.0, init_pos_var);
    std::normal_distribution<double> wide_angle_dist(0.0, init_angle_var);

    for (int i = particles_per_strategy; i < 2 * particles_per_strategy; i++)
    {
        Robot r;
        r.set_states(
            init_x + wide_pos_dist(gen),
            init_y + wide_pos_dist(gen),
            init_theta + wide_angle_dist(gen));
        r.set_noise(0.02, 0.05, 0.02);
        particles_[i] = particle{r, 1.0 / particles_N_};
    }

    // Strategy 3: Uniform grid with small random offset (remaining particles)
    const int grid_size = static_cast<int>(sqrt(particles_N_ - 2 * particles_per_strategy));
    const double grid_span = 0.5; // 50cm total grid span
    const double grid_spacing = grid_span / grid_size;

    int particle_idx = 2 * particles_per_strategy;
    std::uniform_real_distribution<double> small_offset(-grid_spacing * 0.25, grid_spacing * 0.25);
    std::uniform_real_distribution<double> angle_offset(-0.1, 0.1);

    for (int i = 0; i < grid_size && particle_idx < particles_N_; i++)
    {
        for (int j = 0; j < grid_size && particle_idx < particles_N_; j++)
        {
            double x = init_x + (i - grid_size / 2) * grid_spacing + small_offset(gen);
            double y = init_y + (j - grid_size / 2) * grid_spacing + small_offset(gen);
            double theta = init_theta + angle_offset(gen);

            Robot r;
            r.set_states(x, y, theta);
            r.set_noise(0.02, 0.05, 0.02);
            particles_[particle_idx++] = particle{r, 1.0 / particles_N_};
        }
    }

    // Fill any remaining particles with wider distribution
    while (particle_idx < particles_N_)
    {
        Robot r;
        r.set_states(
            init_x + wide_pos_dist(gen) * 1.5,
            init_y + wide_pos_dist(gen) * 1.5,
            init_theta + wide_angle_dist(gen) * 1.5);
        r.set_noise(0.02, 0.05, 0.02);
        particles_[particle_idx++] = particle{r, 1.0 / particles_N_};
    }
}

void MCLNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    pose_ = msg->pose.pose;
}

void MCLNode::estimateCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    initializeMCLComponents(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        tf2::getYaw(msg->pose.pose.orientation));
}

void MCLNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (grid_map_.data.size() == msg->data.size())
        return;

    grid_map_ = *msg;
    int max_map_size = msg->data.size() / 10;
    map_size_ = max_map_size * sizeof(Point);

    if (max_map_size > max_map_N_)
    {
        RCLCPP_ERROR(get_logger(), "Imported map of size %d is too big.", max_map_size);
        return;
    }

    if (map_ != nullptr)
    {
        checkCuda(cudaFree(map_));
    }

    checkCuda(cudaMallocManaged(&map_, map_size_));
    map_N_ = map2arr(grid_map_, map_, max_map_size);
    fill_pc(map_pc_msg_, map_, map_N_, 0, 0, 255, 255, 0.0);
}

void MCLNode::updateMapFromLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Use current pose for map updates
    double robot_x = pose_.position.x; // Using pose_ instead of mcl_msg_
    double robot_y = pose_.position.y;
    double robot_yaw = tf2::getYaw(pose_.orientation);

    // Temporary storage for new points
    std::vector<Point> new_points;
    new_points.reserve(filtered_ranges_.size());

    for (size_t i = 0; i < filtered_ranges_.size(); i++)
    {
        // Calculate angle for this ray
        double angle = laser_min_angle_ + (i * pc_ang_sep_ * M_PI / 180.0) + robot_yaw;

        // Calculate point position in world frame
        double point_x = robot_x + filtered_ranges_[i] * cos(angle);
        double point_y = robot_y + filtered_ranges_[i] * sin(angle);

        // Add to new points
        new_points.push_back(Point{point_x, point_y});

        // Update occupancy grid
        updateGridCell(robot_x, robot_y, point_x, point_y);
    }

    // Update point cloud map
    updatePointCloudMap(new_points);
}

void MCLNode::updateGridCell(double robot_x, double robot_y, double point_x, double point_y)
{
    // Convert to grid coordinates
    int rx = static_cast<int>((robot_x - map_msg_.info.origin.position.x) / map_msg_.info.resolution);
    int ry = static_cast<int>((robot_y - map_msg_.info.origin.position.y) / map_msg_.info.resolution);
    int px = static_cast<int>((point_x - map_msg_.info.origin.position.x) / map_msg_.info.resolution);
    int py = static_cast<int>((point_y - map_msg_.info.origin.position.y) / map_msg_.info.resolution);

    // Use Bresenham's line algorithm to mark free space
    int dx = abs(px - rx);
    int dy = abs(py - ry);
    int sx = rx < px ? 1 : -1;
    int sy = ry < py ? 1 : -1;
    int err = dx - dy;

    int x = rx;
    int y = ry;

    while (true)
    {
        if (x >= 0 && x < map_msg_.info.width && y >= 0 && y < map_msg_.info.height)
        {
            int idx = y * map_msg_.info.width + x;

            // If this is the endpoint
            if (x == px && y == py)
            {
                // Update occupied cell with running average
                if (map_msg_.data[idx] == -1)
                {
                    map_msg_.data[idx] = 100; // First observation
                }
                else
                {
                    map_msg_.data[idx] = (map_msg_.data[idx] + 100) / 2;
                }
            }
            else
            {
                // Update free space with running average
                if (map_msg_.data[idx] == -1)
                {
                    map_msg_.data[idx] = 0; // First observation
                }
                else
                {
                    map_msg_.data[idx] = map_msg_.data[idx] / 2; // Tend towards free
                }
            }
        }

        if (x == px && y == py)
            break;

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }
}

void MCLNode::updatePointCloudMap(const std::vector<Point> &new_points)
{
    // Add new points to map
    for (const auto &point : new_points)
    {
        // Optional: Check for duplicates or close points
        bool is_new = true;
        for (int i = 0; i < map_N_; i++)
        {
            double dx = map_[i].x - point.x;
            double dy = map_[i].y - point.y;
            if (dx * dx + dy * dy < 0.01)
            { // 10cm threshold
                is_new = false;
                break;
            }
        }

        if (is_new && map_N_ < max_map_N_)
        {
            map_[map_N_] = point;
            map_N_++;

            if (map_N_ >= max_map_N_)
            {
                RCLCPP_WARN(get_logger(), "Map reached maximum size (%d points)", max_map_N_);
                break;
            }
        }
    }
    fill_pc(map_pc_msg_, map_, map_N_, 0, 0, 255, 255, 0.0);
}

double normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

void MCLNode::calcControl()
{
    // Calculate displacement in global frame
    double dx = pose_.position.x - last_pose_.position.x;
    double dy = pose_.position.y - last_pose_.position.y;

    // Get yaw angles
    double yaw = tf2::getYaw(pose_.orientation);
    double yaw_last = tf2::getYaw(last_pose_.orientation);

    // Normalize angles to [-π, π]
    while (yaw > M_PI)
        yaw -= 2 * M_PI;
    while (yaw < -M_PI)
        yaw += 2 * M_PI;
    while (yaw_last > M_PI)
        yaw_last -= 2 * M_PI;
    while (yaw_last < -M_PI)
        yaw_last += 2 * M_PI;

    // Calculate turn in the correct direction
    control_.turn = yaw - yaw_last;

    // Normalize turn angle to [-π, π]
    while (control_.turn > M_PI)
        control_.turn -= 2 * M_PI;
    while (control_.turn < -M_PI)
        control_.turn += 2 * M_PI;

    // Calculate distance moved
    double distance = sqrt(dx * dx + dy * dy);

    // If distance is significant, determine forward/backward motion in robot's local frame
    if (distance > 0.001)
    {
        double global_heading = atan2(dy, dx);

        // Normalize angles for comparison
        while (global_heading > M_PI)
            global_heading -= 2 * M_PI;
        while (global_heading < -M_PI)
            global_heading += 2 * M_PI;

        // Calculate heading difference
        double heading_diff = global_heading - yaw_last;
        while (heading_diff > M_PI)
            heading_diff -= 2 * M_PI;
        while (heading_diff < -M_PI)
            heading_diff += 2 * M_PI;

        // If heading difference is more than 90 degrees, robot moved backwards
        if (fabs(heading_diff) > M_PI / 2)
        {
            control_.forward = -distance;
        }
        else
        {
            control_.forward = distance;
        }
    }
    else
    {
        control_.forward = 0.0;
    }

    // Adjust thresholds for minimum movement
    const double min_move = 0.001; // 1mm
    const double min_turn = 0.005; // ~0.3 degrees

    if (fabs(control_.forward) < min_move && fabs(control_.turn) < min_turn)
    {
        control_.forward = 0.0;
        control_.turn = 0.0;
    }
}

void MCLNode::publish_msgs()
{
    auto now = this->get_clock()->now();
    map_pc_msg_.header.stamp = now;
    local_map_pc_msg_.header.stamp = now;
    mcl_msg_.header.stamp = now;
    map_msg_.header.stamp = now;

    map_pc_pub_->publish(map_pc_msg_);
    local_map_pc_pub_->publish(local_map_pc_msg_);
    mcl_pub_->publish(mcl_msg_);
    map_pub_->publish(map_msg_);
}

void MCLNode::update()
{
    publish_msgs();

    RCLCPP_ERROR(get_logger(), "map length: %d", map_N_);
    if (map_ == nullptr || map_N_ == 0)
    {
        RCLCPP_WARN(get_logger(), "Skipping update: Map not loaded yet.");
        return;
    }

    // Compute raycast from current position for visualization
    compute_raycast(map_, map_N_, local_map_, pose_, pc_ang_sep_);
    fill_pc(local_map_pc_msg_, local_map_, local_map_N_,
            255, 0, 0, 255, 0.05);

    // Calculate control from odometry
    calcControl();

    // If there's no significant movement, we might skip the update
    if (fabs(control_.forward) < 0.001 && fabs(control_.turn) < 0.001)
    {
        static int skip_counter = 0;
        skip_counter++;

        // Still update occasionally even with no motion to maintain visualization
        if (skip_counter % 5 != 0)
        {
            RCLCPP_WARN(get_logger(), "Skipping update: No significant motion detected.");

            // Still publish visualizations
            publish_msgs();
            return;
        }
    }

    // Get sensor readings from current position
    robot_.sense(map_, map_N_, local_map_tmp_, local_map_N_,
                 z_, pc_ang_sep_, pose_, true);

    // Run MCL update
    MCL mcl(particles_, particles_N_, control_, z_, local_map_N_,
            map_, map_N_, pc_ang_sep_);
    mcl.resample(belief_);

    // Update particles
    std::swap(particles_, belief_);

    // Calculate and publish mean pose
    mcl_msg_.pose = calculateMeanPose();

    // Save pose for next iteration
    last_pose_ = pose_;

    // Publish results
    publish_msgs();
}

geometry_msgs::msg::PoseWithCovariance MCLNode::calculateMeanPose() const
{
    geometry_msgs::msg::PoseWithCovariance pose_with_cov;

    if (particles_N_ == 0)
        return pose_with_cov;

    // Initialize accumulators
    double sum_weights = 0.0;
    double weighted_x = 0.0;
    double weighted_y = 0.0;
    double weighted_cos = 0.0;
    double weighted_sin = 0.0;

    // First pass: calculate weighted means
    for (int i = 0; i < particles_N_; i++)
    {
        const double weight = particles_[i].w;
        const double x = particles_[i].r.get_x();
        const double y = particles_[i].r.get_y();
        const double theta = particles_[i].r.get_theta();

        weighted_x += x * weight;
        weighted_y += y * weight;
        weighted_cos += std::cos(theta) * weight;
        weighted_sin += std::sin(theta) * weight;
        sum_weights += weight;
    }

    // Handle degenerate case
    if (sum_weights <= 1e-10)
    {
        RCLCPP_WARN(get_logger(), "Warning: Degenerate particle weights detected!");
        sum_weights = 1.0;
        weighted_x = particles_[0].r.get_x();
        weighted_y = particles_[0].r.get_y();
        weighted_cos = std::cos(particles_[0].r.get_theta());
        weighted_sin = std::sin(particles_[0].r.get_theta());
    }

    // Calculate mean position
    const double mean_x = weighted_x / sum_weights;
    const double mean_y = weighted_y / sum_weights;

    // Calculate mean orientation using atan2
    const double mean_cos = weighted_cos / sum_weights;
    const double mean_sin = weighted_sin / sum_weights;
    const double mean_theta = std::atan2(mean_sin, mean_cos);

    // Set mean pose
    pose_with_cov.pose.position.x = mean_x;
    pose_with_cov.pose.position.y = mean_y;

    tf2::Quaternion q;
    q.setRPY(0, 0, mean_theta);
    tf2::convert(q, pose_with_cov.pose.orientation);

    // Calculate covariance matrix
    std::array<double, 36> cov;
    std::fill(cov.begin(), cov.end(), 0.0);

    // Second pass: calculate covariances
    for (int i = 0; i < particles_N_; i++)
    {
        const double w = particles_[i].w / sum_weights;
        const double dx = particles_[i].r.get_x() - mean_x;
        const double dy = particles_[i].r.get_y() - mean_y;
        double dtheta = particles_[i].r.get_theta() - mean_theta;

        // Normalize angle difference to [-π, π]
        while (dtheta > M_PI)
            dtheta -= 2 * M_PI;
        while (dtheta < -M_PI)
            dtheta += 2 * M_PI;

        // Update covariance components
        cov[0] += w * dx * dx;                // Var(x)
        cov[7] += w * dy * dy;                // Var(y)
        cov[35] += w * dtheta * dtheta;       // Var(theta)
        cov[1] = cov[6] += w * dx * dy;       // Cov(x,y)
        cov[5] = cov[30] += w * dx * dtheta;  // Cov(x,theta)
        cov[11] = cov[31] += w * dy * dtheta; // Cov(y,theta)
    }

    // Ensure positive definiteness with minimum variance
    const double min_var = 1e-6;
    for (int i = 0; i < 36; i += 7)
    { // Diagonal elements
        cov[i] = std::max(cov[i], min_var);
    }

    std::copy(cov.begin(), cov.end(), pose_with_cov.covariance.begin());
    return pose_with_cov;
}

void MCLNode::updateMapFromRaycast(
    nav_msgs::msg::OccupancyGrid &map_msg,
    const geometry_msgs::msg::Pose &robot_pose,
    const Point *ray_endpoints,
    int num_rays,
    double max_range) // Maximum sensor range
{
    // Bresenham's line algorithm for ray tracing
    auto plotLine = [&map_msg](int x0, int y0, int x1, int y1, int8_t value)
    {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int x = x0;
        int y = y0;
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            // Check bounds
            if (x >= 0 && x < map_msg.info.width &&
                y >= 0 && y < map_msg.info.height)
            {
                // Only update if cell is unknown or has lower confidence
                int idx = y * map_msg.info.width + x;
                if (map_msg.data[idx] == -1 ||
                    (value == 0 && map_msg.data[idx] > 0 && map_msg.data[idx] < 90) ||
                    (value == 100 && map_msg.data[idx] < 10))
                {
                    map_msg.data[idx] = value;
                }
            }

            if (x == x1 && y == y1)
                break;
            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y += sy;
            }
        }
    };

    // Get robot position in grid coordinates
    int robot_x = static_cast<int>((robot_pose.position.x - map_msg.info.origin.position.x) / map_msg.info.resolution);
    int robot_y = static_cast<int>((robot_pose.position.y - map_msg.info.origin.position.y) / map_msg.info.resolution);

    // Process each ray
    for (int i = 0; i < num_rays; i++)
    {
        // Convert endpoint to grid coordinates
        int end_x = static_cast<int>((ray_endpoints[i].x - map_msg.info.origin.position.x) / map_msg.info.resolution);
        int end_y = static_cast<int>((ray_endpoints[i].y - map_msg.info.origin.position.y) / map_msg.info.resolution);

        // Calculate distance to endpoint
        double dx = ray_endpoints[i].x - robot_pose.position.x;
        double dy = ray_endpoints[i].y - robot_pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance <= max_range)
        {
            // Mark cells along the ray as free (0)
            plotLine(robot_x, robot_y, end_x, end_y, 0);

            // Mark the endpoint as occupied (100)
            if (end_x >= 0 && end_x < map_msg.info.width &&
                end_y >= 0 && end_y < map_msg.info.height)
            {
                map_msg.data[end_y * map_msg.info.width + end_x] = 100;
            }
        }
    }
}

// Helper function to expand the map if needed
void MCLNode::expandMapIfNeeded(
    nav_msgs::msg::OccupancyGrid &map_msg,
    const geometry_msgs::msg::Pose &robot_pose,
    double expansion_margin)
{
    // Check if robot is too close to map boundaries
    double robot_x = robot_pose.position.x;
    double robot_y = robot_pose.position.y;

    // Get map boundaries in world coordinates
    double map_min_x = map_msg.info.origin.position.x;
    double map_min_y = map_msg.info.origin.position.y;
    double map_max_x = map_min_x + map_msg.info.width * map_msg.info.resolution;
    double map_max_y = map_min_y + map_msg.info.height * map_msg.info.resolution;

    // Calculate required expansion in each direction
    int expand_left = 0, expand_right = 0, expand_up = 0, expand_down = 0;

    if (robot_x - map_min_x < expansion_margin)
    {
        expand_left = static_cast<int>((expansion_margin - (robot_x - map_min_x)) / map_msg.info.resolution) + 1;
    }
    if (map_max_x - robot_x < expansion_margin)
    {
        expand_right = static_cast<int>((expansion_margin - (map_max_x - robot_x)) / map_msg.info.resolution) + 1;
    }
    if (robot_y - map_min_y < expansion_margin)
    {
        expand_down = static_cast<int>((expansion_margin - (robot_y - map_min_y)) / map_msg.info.resolution) + 1;
    }
    if (map_max_y - robot_y < expansion_margin)
    {
        expand_up = static_cast<int>((expansion_margin - (map_max_y - robot_y)) / map_msg.info.resolution) + 1;
    }

    // If no expansion needed, return
    if (expand_left == 0 && expand_right == 0 && expand_up == 0 && expand_down == 0)
    {
        return;
    }

    // Create new data array
    uint32_t new_width = map_msg.info.width + expand_left + expand_right;
    uint32_t new_height = map_msg.info.height + expand_up + expand_down;
    std::vector<int8_t> new_data(new_width * new_height, -1);

    // Copy existing data to new array
    for (uint32_t y = 0; y < map_msg.info.height; ++y)
    {
        for (uint32_t x = 0; x < map_msg.info.width; ++x)
        {
            uint32_t old_idx = y * map_msg.info.width + x;
            uint32_t new_idx = (y + expand_down) * new_width + (x + expand_left);
            new_data[new_idx] = map_msg.data[old_idx];
        }
    }

    // Update map properties
    map_msg.info.origin.position.x -= expand_left * map_msg.info.resolution;
    map_msg.info.origin.position.y -= expand_down * map_msg.info.resolution;
    map_msg.info.width = new_width;
    map_msg.info.height = new_height;
    map_msg.data = std::move(new_data);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MCLNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}