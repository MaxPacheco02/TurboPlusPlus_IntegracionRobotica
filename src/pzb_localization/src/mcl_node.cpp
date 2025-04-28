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
    particles_N_ = 500;
    pc_ang_sep_ = 9.0;   // Reduced from 9.0 for better angular resolution
    update_rate_ = 50;  // Hz
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

    // Get yaw angles and normalize them
    double yaw = tf2::getYaw(pose_.orientation);
    double yaw_last = tf2::getYaw(last_pose_.orientation);
    
    // Calculate motion in robot's local frame
    double global_heading = atan2(dy, dx);
    
    // Calculate turn first (in local frame)
    control_.turn = normalize_angle(yaw - yaw_last);
    
    // Calculate forward motion
    control_.forward = sqrt(dx*dx + dy*dy);

    // Adjust thresholds for minimum movement
    const double min_move = 0.0001;  // 0.1mm - reduced from 1mm
    const double min_turn = 0.001;   // ~0.057 degrees
    
    if (control_.forward < min_move && fabs(control_.turn) < min_turn) {
        // If both motion components are very small, treat as stationary
        control_.forward = 0.0;
        control_.turn = 0.0;
    } else {
        // Otherwise keep the small motions to maintain particle diversity
        if (control_.forward < min_move) control_.forward = min_move;
        if (fabs(control_.turn) < min_turn) {
            control_.turn = (control_.turn >= 0) ? min_turn : -min_turn;
        }
    }

    // Debug output
    RCLCPP_DEBUG(this->get_logger(), 
        "Control update: forward=%.6f, turn=%.6f (rad) = %.2f deg",
        control_.forward, control_.turn, control_.turn * 180.0 / M_PI);
    RCLCPP_DEBUG(this->get_logger(),
        "Pose delta: dx=%.6f, dy=%.6f, dyaw=%.6f",
        dx, dy, control_.turn);
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
    
    if (particles_N_ == 0) return pose_with_cov;

    // Initialize accumulators
    double sum_weights = 0.0;
    double weighted_x = 0.0;
    double weighted_y = 0.0;
    double weighted_cos = 0.0;
    double weighted_sin = 0.0;

    // First pass: calculate weighted means
    for (int i = 0; i < particles_N_; i++) {
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
    if (sum_weights <= 1e-10) {
        sum_weights = 1.0;
        // Use first particle as fallback
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
    for (int i = 0; i < particles_N_; i++) {
        const double w = particles_[i].w / sum_weights;
        const double dx = particles_[i].r.get_x() - mean_x;
        const double dy = particles_[i].r.get_y() - mean_y;
        double dtheta = particles_[i].r.get_theta() - mean_theta;
        
        // Normalize angle difference to [-π, π]
        while (dtheta > M_PI) dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;

        // Update covariance components
        cov[0] += w * dx * dx;        // Var(x)
        cov[7] += w * dy * dy;        // Var(y)
        cov[35] += w * dtheta * dtheta; // Var(theta)
        cov[1] = cov[6] += w * dx * dy; // Cov(x,y)
        cov[5] = cov[30] += w * dx * dtheta; // Cov(x,theta)
        cov[11] = cov[31] += w * dy * dtheta; // Cov(y,theta)
    }

    // Ensure positive definiteness with minimum variance
    const double min_var = 1e-6;
    for (int i = 0; i < 36; i += 7) {  // Diagonal elements
        cov[i] = std::max(cov[i], min_var);
    }

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