#include "robot.h"

double Robot::forward_noise__;
double Robot::turn_noise__;
double Robot::sense_noise__;

Robot::Robot() : x__(0.0), y__(0.0), orientation__(0.0) {
    update_pose_msg();
}

void Robot::set_states(double new_x, double new_y, double new_orient) {
    x__ = new_x;
    y__ = new_y;
    orientation__ = new_orient;
    update_pose_msg();
}

void Robot::set_noise(double new_f_noise, double new_t_noise, double new_s_noise) {
    forward_noise__ = new_f_noise;
    turn_noise__ = new_t_noise;
    sense_noise__ = new_s_noise;
}

__host__ __device__ void Robot::sense(
    const Point* map, int map_N,
    Point* local_map, int local_map_N,
    double* sensor, double ang_sep,
    geometry_msgs::msg::Pose ext_pose,
    bool add_noise) 
{
    // Compute raycast from current position
    compute_raycast(map, map_N, local_map, ext_pose, ang_sep);

    // Calculate distances for each raycast point
    for (int i = 0; i < local_map_N; i++) {
        // Get Euclidean distance to each landmark
        double dx = local_map[i].x - ext_pose.position.x;
        double dy = local_map[i].y - ext_pose.position.y;
        double dist = sqrt(dx * dx + dy * dy);

        if (add_noise) {
            // Add realistic noise model:
            // - Proportional to distance (longer ranges have more noise)
            // - With a minimum floor value
            double noise_std = std::max(0.02, 0.01 * dist); // 1% of distance or 2cm, whichever is larger
            
            #ifdef __CUDA_ARCH__
            // Device code path
            dist += utility::get_gaussian_random_number(0.0, noise_std);
            #else
            // Host code path
            double noise = utility::get_gaussian_random_number(0.0, noise_std);
            // Bound the noise to prevent extreme values
            dist += noise;
            #endif
        }
        
        sensor[i] = dist;
        
        #ifndef __CUDA_ARCH__
        if (i < 5) { // Log first few measurements for debugging
            std::cout << "Sensor " << i << ": distance = " << dist << std::endl;
        }
        #endif
    }
}

__host__ __device__ void Robot::move(double turn, double forward) {
    // Add noise to motion
    double noisy_turn = turn;
    double noisy_forward = forward;

    #ifdef __CUDA_ARCH__
    noisy_turn += utility::get_gaussian_random_number(0.0, turn_noise__);
    noisy_forward += utility::get_gaussian_random_number(0.0, forward_noise__);
    #else
    noisy_turn += utility::get_gaussian_random_number(0.0, turn_noise__);
    noisy_forward += utility::get_gaussian_random_number(0.0, forward_noise__);
    #endif

    // Update orientation
    orientation__ = fmod((orientation__ + noisy_turn), (2 * M_PI));
    if (orientation__ < 0) {
        orientation__ += 2 * M_PI;
    }

    // Update position
    x__ += noisy_forward * cos(orientation__);
    y__ += noisy_forward * sin(orientation__);

    #ifndef __CUDA_ARCH__
    update_pose_msg();
    #endif
}

void Robot::update_pose_msg() {
    pose__.position.x = x__;
    pose__.position.y = y__;

    tf2::Quaternion q;
    q.setRPY(0, 0, orientation__);
    tf2::convert(q, pose__.orientation);
}

std::string Robot::get_pose() const {
    return std::string("[X=") + std::to_string(x__) +
           std::string(", Y=") + std::to_string(y__) +
           std::string(" Theta=") + std::to_string(orientation__) +
           std::string("]");
}