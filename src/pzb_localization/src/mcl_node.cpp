#include "mcl_node.h"
#include <chrono>

using namespace std::chrono_literals;

MCLNode::MCLNode() : Node("mcl_node")
{
    initializeParameters();
    initializePublishersAndSubscribers();
    initializeMessages();
    initializeMCLComponents(0.0, 0.0, 0.0);
    map_ = nullptr;
    map_N_ = 0;
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

void MCLNode::initializeParameters()
{
    particles_N_ = 500;
    pc_ang_sep_ = 9.0; // Reduced from 9.0 for better angular resolution
    update_rate_ = 50; // Hz
}

void MCLNode::initializePublishersAndSubscribers()
{
    // Create publishers
    map_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pzb/map_pc", 10);
    local_map_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pzb/pc", 10);
    mcl_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/mcl/odom", 10);

    // Create subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/pzb/odom", 10,
        std::bind(&MCLNode::odomCallback, this, std::placeholders::_1));

    initial_estimate_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        std::bind(&MCLNode::estimateCallback, this, std::placeholders::_1));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/pzb/map", 10,
        std::bind(&MCLNode::mapCallback, this, std::placeholders::_1));

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

    std::cout << "Initializing particles around pose: "
              << "x=" << pose_.position.x
              << ", y=" << pose_.position.y
              << ", yaw=" << tf2::getYaw(pose_.orientation) << std::endl;

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
    std::uniform_real_distribution<double> angle_offset(-0.1, 0.1); // ~±5.7 degrees

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

    // Print first few particles for debugging
    std::cout << "Initial particle distribution (first 5):" << std::endl;
    for (int i = 0; i < 5; i++)
    {
        std::cout << "Particle " << i << ": x=" << particles_[i].r.get_x()
                  << ", y=" << particles_[i].r.get_y()
                  << ", theta=" << particles_[i].r.get_theta() << std::endl;
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

    if (map_ != nullptr)
    {
        checkCuda(cudaFree(map_));
    }

    checkCuda(cudaMallocManaged(&map_, map_size_));
    map_N_ = map2arr(grid_map_, map_, max_map_size);
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

    // Debug output
    std::cout << "\nControl update details:" << std::endl;
    std::cout << "Current pose: x=" << pose_.position.x
              << ", y=" << pose_.position.y
              << ", yaw=" << yaw << std::endl;
    std::cout << "Last pose: x=" << last_pose_.position.x
              << ", y=" << last_pose_.position.y
              << ", yaw=" << yaw_last << std::endl;

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

    // Debug deltas
    std::cout << "Delta: dx=" << dx << ", dy=" << dy
              << ", dyaw=" << control_.turn << std::endl;

    // Adjust thresholds for minimum movement
    const double min_move = 0.001; // 1mm
    const double min_turn = 0.005; // ~0.3 degrees

    if (fabs(control_.forward) < min_move && fabs(control_.turn) < min_turn)
    {
        control_.forward = 0.0;
        control_.turn = 0.0;
    }

    std::cout << "Control: forward=" << control_.forward
              << ", turn=" << control_.turn << std::endl;
}

void MCLNode::update()
{
    if (map_ == nullptr || map_N_ == 0)
    {
        std::cout << "Skipping update: Map not loaded yet" << std::endl;
        return;
    }

    // Debug pose info
    std::cout << "\n---- MCL Update ----" << std::endl;
    std::cout << "Current pose: x=" << pose_.position.x
              << ", y=" << pose_.position.y
              << ", yaw=" << tf2::getYaw(pose_.orientation) << std::endl;
    std::cout << "Last pose: x=" << last_pose_.position.x
              << ", y=" << last_pose_.position.y
              << ", yaw=" << tf2::getYaw(last_pose_.orientation) << std::endl;

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
            std::cout << "Skipping update: No significant motion detected" << std::endl;

            // Still publish visualizations
            auto now = this->get_clock()->now();
            map_pc_msg_.header.stamp = now;
            local_map_pc_msg_.header.stamp = now;
            mcl_msg_.header.stamp = now;
            map_pc_pub_->publish(map_pc_msg_);
            local_map_pc_pub_->publish(local_map_pc_msg_);
            mcl_pub_->publish(mcl_msg_);

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
    auto now = this->get_clock()->now();
    map_pc_msg_.header.stamp = now;
    local_map_pc_msg_.header.stamp = now;
    mcl_msg_.header.stamp = now;

    map_pc_pub_->publish(map_pc_msg_);
    local_map_pc_pub_->publish(local_map_pc_msg_);
    mcl_pub_->publish(mcl_msg_);
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

    // Debug sample of initial particles
    std::cout << "\nParticle sample (first 5 before mean calculation):" << std::endl;
    for (int i = 0; i < std::min(5, particles_N_); i++)
    {
        std::cout << "Particle " << i << ": x=" << particles_[i].r.get_x()
                  << ", y=" << particles_[i].r.get_y()
                  << ", theta=" << particles_[i].r.get_theta()
                  << ", w=" << particles_[i].w << std::endl;
    }

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
        std::cout << "Warning: Degenerate particle weights detected!" << std::endl;
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

    std::cout << "Calculated mean pose: x=" << mean_x
              << ", y=" << mean_y
              << ", theta=" << mean_theta
              << ", sum_weights=" << sum_weights << std::endl;

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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MCLNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}