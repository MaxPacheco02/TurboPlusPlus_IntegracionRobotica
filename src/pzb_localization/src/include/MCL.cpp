#include "MCL.h"

MCL::MCL(particle *X, int X_N, geometry_msgs::msg::Transform tf_cmd, double *Z, int Z_N, Point *map, int map_N, double ang_sep)
    : X__(X), X_N__(X_N), tf_cmd__(tf_cmd), Z__(Z), Z_N__(Z_N), map__(map), map_N__(map_N),
      ang_sep__(ang_sep), max_weight__(0.0) {}

void MCL::motion_update()
{
    for (int i = 0; i < X_N__; i++)
    {
        X__[i].r.move(tf_cmd__);
    }
}

void MCL::sensor_update()
{
    // keep track of maximum weight for resampling
    max_weight__ = 0.0;
    // update weights with probability of each observations using pdf

    Point *local_map;
    size_t local_map_size = Z_N__ * sizeof(Point);
    checkCuda(cudaMallocManaged(&local_map, local_map_size));

    double *meas;
    checkCuda(cudaMallocManaged(&meas, local_map_size));

    for (int i = 0; i < X_N__; i++)
    {
        // sense and calculate probability of robot measurements Z considering a pdf of mean observed distance from map and variance of sensor noise
        X__[i].r.sense(map__, map_N__, local_map, Z_N__, meas, ang_sep__, X__[i].r.get_pose_msg(), false);
        X__[i].w = utility::get_measurement_prob(meas, Z__, Z_N__, X__[0].r.get_sensor_noise());

        // update max weight
        X__[i].w > max_weight__ ? max_weight__ = X__[i].w : max_weight__ += 0.0;
    }

    checkCuda(cudaFree(local_map));
    checkCuda(cudaFree(meas));
}

void MCL::resample(particle *new_belief)
{
    // move particles
    motion_update();
    // sense
    sensor_update();

    // Debug: Print some weights
    std::cout << "Sample weights after sensor update:" << std::endl;
    for (int i = 0; i < std::min(5, X_N__); i++)
    {
        std::cout << "Particle " << i << ": weight = " << X__[i].w << std::endl;
    }
    std::cout << "Max weight: " << max_weight__ << std::endl;

    // use resampling wheel algorithm to resample particles
    cp_particle_from_to(X__, new_belief, X_N__);
    // get random index
    int index = utility::get_random_number() * X_N__;
    double beta = 0.0;
    for (int i = 0; i < X_N__; ++i)
    {
        beta += utility::get_random_number() * 2 * max_weight__;
        while (beta > X__[index].w)
        {
            beta -= X__[index].w;
            index = (index + 1) % X_N__;
        }
        //  select particle at index
        new_belief[i] = X__[index];
    }
}
