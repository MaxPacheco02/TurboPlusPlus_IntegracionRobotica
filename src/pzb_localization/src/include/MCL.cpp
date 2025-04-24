#include "MCL.h"
#include "utility.h"
#include <assert.h>
#include "cuda_kernels.h"

MCL::MCL(particle *X, int X_N, command U, double *Z, int Z_N, Point *map, int map_N) : 
    X__(X), X_N__(X_N), U__(U), Z__(Z), Z_N__(Z_N), map__(map), map_N__(map_N), max_weight__(0.0) {}

void MCL::motion_update()
{
    motion_nvcc(X__, X_N__, U__);
}

void MCL::sensor_update()
{
    // keep track of maximum weight for resampling
    max_weight__ = 0.0;
    // update weights with probability of each observations using pdf
    std::for_each(X__.begin(), X__.end(), [this](particle &ptc)
                  {
        // sense environemnt with each particle
        std::vector<double> meas = ptc.r.sense(map__,ptc.r.get_pose_msg(),false);
        // calculate probability of robot measurements Z considering a pdf of mean observed distance from map and variance of sensor noise
        ptc.w = utility::get_measurement_prob(meas);
        // update max weight
        ptc.w > max_weight__ ? max_weight__ = ptc.w : max_weight__ += 0.0; });
}


std::vector<particle> MCL::resample()
{
    // move particles
    motion_update();
    // sense
    sensor_update();
    // use resampling wheel algorithm to resample particles
    std::vector<particle> p = X__;
    // get random index
    int index = utility::get_random_number() * X__.size();
    double beta = 0.0;
    for (int i = 0; i < X__.size(); ++i)
    {
        beta += utility::get_random_number() * 2 * max_weight__;
        while (beta > X__[index].w)
        {
            beta -= X__[index].w;
            index = (index + 1) % X__.size();
        }
        // std::cout << index << std::endl;
        //  select particle at index
        p[i] = X__[index];
        // std::cout << p[i].r.get_pose() << std::endl;
    }
    // return new particle set
    return p;
}
