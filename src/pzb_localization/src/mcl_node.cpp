#include "mcl_node.h"
#include <chrono>

using namespace std::chrono_literals;

MCLNode::MCLNode() : Node("mcl_node") {
    initializeParameters();
    initializePublishersAndSubscribers();
    initializeMessages();
    initializeMCLComponents();
}

MCLNode::~MCLNode() {
    if (map_ != nullptr) cudaFree(map_);
    if (local_map_ != nullptr) cudaFree(local_map_);
    if (particles_ != nullptr) cudaFree(particles_);
    if (belief_ != nullptr) cudaFree(belief_);
    if (z_ != nullptr) cudaFree(z_);
    if (local_map_tmp_ != nullptr) cudaFree(local_map_tmp_);
}

void MCLNode::initializeParameters() {
    particles_N_ = 1000;
    pc_ang_sep_ = 9.0;  // degrees
    update_rate_ = 100; // Hz
}

void MCLNode::initializePublishersAndSubscribers() {
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

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/pzb/map", 10,
        std::bind(&MCLNode::mapCallback, this, std::placeholders::_1));

    // Create timer
    timer_ = create_wall_timer(
        std::chrono::milliseconds(1000/update_rate_),
        std::bind(&MCLNode::update, this));
}

void MCLNode::initializeMessages() {
    init_pc_msg(map_pc_msg_);
    init_pc_msg(local_map_pc_msg_);
    mcl_msg_.header.frame_id = "odom";
    
    // Initialize poses
    pose_.position = geometry_msgs::build<geometry_msgs::msg::Point>()
        .x(0.0).y(0.0).z(0.0);
    pose_.orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>()
        .x(0.0).y(0.0).z(0.0).w(1.0);
    last_pose_ = pose_;
}

void MCLNode::initializeMCLComponents() {
    // Initialize robot
    robot_.set_states(0.0, 0.0, 0.0);
    robot_.set_noise(0.05, 0.05, 0.1);

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

    // Initialize particles
    particle initial_particle{robot_, 1.0/particles_N_};
    for (int i = 0; i < particles_N_; i++) {
        particles_[i] = initial_particle;
    }

    map_ = nullptr;
    map_N_ = 0;
}

void MCLNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    pose_ = msg->pose.pose;
}

void MCLNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (grid_map_.data.size() == msg->data.size()) return;

    grid_map_ = *msg;
    int max_map_size = msg->data.size() / 10;
    map_size_ = max_map_size * sizeof(Point);
    
    if (map_ != nullptr) {
        checkCuda(cudaFree(map_));
    }
    
    checkCuda(cudaMallocManaged(&map_, map_size_));
    map_N_ = map2arr(grid_map_, map_, max_map_size);
    fill_pc(map_pc_msg_, map_, map_N_, 0, 0, 255, 255, 0.0);
}

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void MCLNode::calcControl() {
    // Calculate displacement in global frame
    double dx = pose_.position.x - last_pose_.position.x;
    double dy = pose_.position.y - last_pose_.position.y;

    // Get yaw angles
    double yaw = tf2::getYaw(pose_.orientation);
    double yaw_last = tf2::getYaw(last_pose_.orientation);

    // Calculate motion in robot's local frame
    control_.forward = sqrt(dx*dx + dy*dy);
    control_.turn = normalize_angle(yaw - yaw_last);

    // Threshold for minimum movement to consider
    const double min_move = 0.001;  // 1mm
    const double min_turn = 0.001;  // ~0.057 degrees

    if (control_.forward < min_move) control_.forward = 0.0;
    if (fabs(control_.turn) < min_turn) control_.turn = 0.0;

    // Detailed debug output
    std::cout << "\nControl update details:" << std::endl;
    std::cout << "Current pose: x=" << pose_.position.x 
              << ", y=" << pose_.position.y 
              << ", yaw=" << yaw << std::endl;
    std::cout << "Last pose: x=" << last_pose_.position.x 
              << ", y=" << last_pose_.position.y 
              << ", yaw=" << yaw_last << std::endl;
    std::cout << "Delta: dx=" << dx << ", dy=" << dy 
              << ", dyaw=" << (yaw - yaw_last) << std::endl;
    std::cout << "Control: forward=" << control_.forward 
              << ", turn=" << control_.turn << std::endl;
}

void MCLNode::update() {
    if (map_ == nullptr || map_N_ == 0) return;

    // Debug last_pose_ update
    std::cout << "\nBefore last_pose_ update:" << std::endl;
    std::cout << "last_pose_: x=" << last_pose_.position.x 
              << ", y=" << last_pose_.position.y 
              << ", yaw=" << tf2::getYaw(last_pose_.orientation) << std::endl;
    std::cout << "current pose_: x=" << pose_.position.x 
              << ", y=" << pose_.position.y 
              << ", yaw=" << tf2::getYaw(pose_.orientation) << std::endl;

    // Update visualization
    compute_raycast(map_, map_N_, local_map_, pose_, pc_ang_sep_);
    fill_pc(local_map_pc_msg_, local_map_, local_map_N_, 
            255, 0, 0, 255, 0.05);

    // Calculate control from odometry
    calcControl();

    // Get sensor readings
    robot_.sense(map_, map_N_, local_map_tmp_, local_map_N_, 
                z_, pc_ang_sep_, pose_, true);

    // Create MCL instance and update
    MCL mcl(particles_, particles_N_, control_, z_, local_map_N_,
            map_, map_N_, pc_ang_sep_);
    mcl.resample(belief_);

    // Update particles
    std::swap(particles_, belief_);

    // Calculate and publish mean pose
    mcl_msg_.pose = calculateMeanPose();
    
    // Save pose for next iteration
    last_pose_ = pose_;

    std::cout << "After last_pose_ update:" << std::endl;
    std::cout << "last_pose_: x=" << last_pose_.position.x 
              << ", y=" << last_pose_.position.y 
              << ", yaw=" << tf2::getYaw(last_pose_.orientation) << std::endl;
              
    // Publish results
    auto now = this->get_clock()->now();
    map_pc_msg_.header.stamp = now;
    local_map_pc_msg_.header.stamp = now;
    mcl_msg_.header.stamp = now;

    map_pc_pub_->publish(map_pc_msg_);
    local_map_pc_pub_->publish(local_map_pc_msg_);
    mcl_pub_->publish(mcl_msg_);
}

geometry_msgs::msg::PoseWithCovariance MCLNode::calculateMeanPose() const {
    geometry_msgs::msg::PoseWithCovariance pose_with_cov;
    
    if (particles_N_ == 0) {
        return pose_with_cov;
    }

    // Calculate weighted mean using stable numerics
    double total_weight = 0.0;
    double x_weighted_sum = 0.0;
    double y_weighted_sum = 0.0;
    double cos_weighted_sum = 0.0;
    double sin_weighted_sum = 0.0;

    for (int i = 0; i < particles_N_; i++) {
        const double weight = particles_[i].w;
        const double x = particles_[i].r.get_x();
        const double y = particles_[i].r.get_y();
        const double theta = particles_[i].r.get_theta();

        x_weighted_sum += x * weight;
        y_weighted_sum += y * weight;
        cos_weighted_sum += std::cos(theta) * weight;
        sin_weighted_sum += std::sin(theta) * weight;
        total_weight += weight;
    }

    // Handle zero total weight case
    if (total_weight <= 0.0) {
        // Use first particle as fallback
        geometry_msgs::msg::Point position;
        position.x = particles_[0].r.get_x();
        position.y = particles_[0].r.get_y();

        tf2::Quaternion q;
        q.setRPY(0, 0, particles_[0].r.get_theta());
        geometry_msgs::msg::Quaternion orientation;
        tf2::convert(q, orientation);

        pose_with_cov.pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
            .position(position)
            .orientation(orientation);
        return pose_with_cov;
    }

    // Calculate mean values
    const double inv_weight = 1.0 / total_weight;
    const double x_mean = x_weighted_sum * inv_weight;
    const double y_mean = y_weighted_sum * inv_weight;
    const double theta_mean = std::atan2(
        sin_weighted_sum * inv_weight,
        cos_weighted_sum * inv_weight
    );

    // Set mean pose
    geometry_msgs::msg::Point position;
    position.x = x_mean;
    position.y = y_mean;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_mean);
    geometry_msgs::msg::Quaternion orientation;
    tf2::convert(q, orientation);

    pose_with_cov.pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(position)
        .orientation(orientation);

    // Calculate covariance matrix
    std::array<double, 36> cov;
    std::fill(cov.begin(), cov.end(), 0.0);

    for (int i = 0; i < particles_N_; i++) {
        const double x = particles_[i].r.get_x();
        const double y = particles_[i].r.get_y();
        const double theta = particles_[i].r.get_theta();
        const double normalized_weight = particles_[i].w * inv_weight;

        // Handle angle difference properly
        double theta_diff = theta - theta_mean;
        if (theta_diff > M_PI) theta_diff -= 2 * M_PI;
        if (theta_diff < -M_PI) theta_diff += 2 * M_PI;

        const double dx = x - x_mean;
        const double dy = y - y_mean;

        // Update covariance components
        cov[0] += normalized_weight * dx * dx;       // x variance
        cov[1] += normalized_weight * dx * dy;       // x-y covariance
        cov[6] = cov[1];                            // y-x covariance
        cov[7] += normalized_weight * dy * dy;       // y variance
        cov[5] += normalized_weight * dx * theta_diff;  // x-yaw covariance
        cov[30] = cov[5];                           // yaw-x covariance
        cov[11] += normalized_weight * dy * theta_diff; // y-yaw covariance
        cov[31] = cov[11];                          // yaw-y covariance
        cov[35] += normalized_weight * theta_diff * theta_diff; // yaw variance
    }

    // Ensure positive definiteness
    constexpr double min_variance = 1e-6;
    cov[0] = std::max(cov[0], min_variance);   // x variance
    cov[7] = std::max(cov[7], min_variance);   // y variance
    cov[14] = min_variance;                    // z variance
    cov[21] = min_variance;                    // roll variance
    cov[28] = min_variance;                    // pitch variance
    cov[35] = std::max(cov[35], min_variance); // yaw variance

    // Set covariance matrix
    std::copy(cov.begin(), cov.end(), pose_with_cov.covariance.begin());

    return pose_with_cov;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MCLNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}