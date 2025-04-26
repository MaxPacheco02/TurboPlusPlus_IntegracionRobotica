#include "robot.h"

double Robot::forward_noise__;
double Robot::turn_noise__;
double Robot::sense_noise__;

Robot::Robot() {}

void Robot::set_states(double new_x, double new_y, double new_orient)
{
    // replace robot states with new values
    x__ = new_x;
    y__ = new_y;
    orientation__ = new_orient;

    update_pose_msg();
}

void Robot::set_noise(double new_f_noise, double new_t_noise, double new_s_noise)
{
    // set noises
    forward_noise__ = new_f_noise;
    turn_noise__ = new_t_noise;
    sense_noise__ = new_s_noise;
}

__host__ __device__ void Robot::sense(const Point *map, int map_N, Point* local_map, int local_map_N, double *sensor, double ang_sep, geometry_msgs::msg::Pose ext_pose, bool noise)
{
    compute_raycast(map, map_N, local_map, ext_pose, ang_sep);

    std::cout << "Sample sensor readings:" << std::endl;

    // iterate through pointcloud, to add noise
    for (int i = 0; i < local_map_N; i++)
    {
        // get Euclidean distance to each landmark and add noise to simulate range finder data
        double m = sqrt(pow((local_map[i].x - ext_pose.position.x), 2) + pow((local_map[i].y - ext_pose.position.y), 2));
        std::cout << "Sensor " << i << ": distance = " << m;
        noise ? m += utility::get_gaussian_random_number(0.0, sense_noise__) : m += 0.0;
        sensor[i] = m;
        std::cout << ", with noise = " << sensor[i] << std::endl;
    }

}

__host__ __device__ void Robot::move(double turn, double forward)
{
    // set rotation, add gaussian noise with mean of rotation bias and turn_noise as variance
    // here we assume trn bias is zero
    orientation__ = fmod((orientation__ + turn + utility::get_gaussian_random_number(0.0, turn_noise__)), (2 * M_PI));

    double dist = forward + utility::get_gaussian_random_number(0, forward_noise__);
    x__ += dist * cos(orientation__);
    y__ += dist * sin(orientation__);

    update_pose_msg();
}

void Robot::update_pose_msg()
{
    pose__.position.x = x__;
    pose__.position.y = y__;

    tf2::Quaternion q;
    q.setRPY(0, 0, orientation__); // Roll, Pitch, Yaw
    tf2::convert(q, pose__.orientation);
}

std::string Robot::get_pose()
{
    std::string pose = std::string("[") + std::string("X=") + std::to_string(x__) + std::string(", Y=") + std::to_string(y__) + std::string(" Theta=") + std::to_string(orientation__) + std::string("]");
    return pose;
}

__host__ __device__ double Robot::get_sensor_noise() { return sense_noise__; }

double Robot::get_x() const { return x__; }

double Robot::get_y() const { return y__; }

double Robot::get_theta() const { return orientation__; }

__host__ __device__ geometry_msgs::msg::Pose Robot::get_pose_msg() const { return pose__; };