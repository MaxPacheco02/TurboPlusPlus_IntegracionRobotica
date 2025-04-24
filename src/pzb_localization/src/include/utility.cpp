#include "utility.h"
#include "MCL.h"

#include <iostream>
#include <math.h>
#include <random>

double utility::get_gaussian_random_number(double mean, double var)
{
    // Random Generators
    std::random_device rd;
    std::mt19937 gen(rd());
    // get rangom number from normal distribution
    std::normal_distribution<double> dist(mean, var);
    return dist(gen);
}

double utility::get_random_number()
{
    // Random Generators
    std::random_device rd;
    std::mt19937 gen(rd());
    // get random number from a uniform distribution
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}

double utility::get_gaussian_probability(double mean, double var, double x)
{
    //std::cout << mean << " " << var << " " << x << std::endl;
    // Probability of x given normal ditribution with mean and variance
    double p = exp(-(pow((mean - x), 2)) / (pow(var, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(var, 2)));
    //std::cout << p << std::endl;
    return p;
}

double utility::get_measurement_prob(double* measurement, int meas_N, double* sensor, int sensor_N, double var)
{
    if(meas_N != sensor_N){
        return -1;
    }

    double prob = 1.0;

    for(int i = 0 ; i < meas_N ; i++){
        prob*= utility::get_gaussian_probability(measurement[i], var, sensor[i]);
    }

    return prob;
}


double utility::evaluation(Robot *r, std::vector<particle> *p)
{
    //Calculate the mean error of the system
    double sum = 0.0;
    for (int i = 0; i < p->size(); i++)
    {
        double dx = (*p)[i].r.get_x() - r->get_x();
        double dy = (*p)[i].r.get_y() - r->get_y();
        double err = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += err;
    }
    return sum / p->size();
}

void utility::visualization(Robot *robot, int step, std::vector<particle> *belief, std::vector<particle> *new_belief, std::vector<Point>& map)
{
    float x_min = 0.0;
    float y_min = 0.0;
    float x_max = 5.0;
    float y_max = 3.0;

    //Draw cvplot scatter plot for the robot, landmarks, particles and resampled particles on a graph
    int n = belief->size();
    std::vector<std::pair<float, float>> data;

    //Graph Format
    auto name = "MCL";
    cvplot::setWindowTitle(name, "step" + std::to_string(step));
    cvplot::moveWindow(name, 0, 0);
    cvplot::resizeWindow(name, 800, 600);
    auto &figure = cvplot::figure(name);
    figure.origin(true, true);
    figure.setAxes(x_min, y_min, x_max, y_max);

    //Draw robot position in blue
    data.clear();
    data.push_back({robot->get_x(), robot->get_y()});
    figure.series("Robot").set(data).type(cvplot::Dots).color(cvplot::Black);

    //Draw resampled particles in yellow
    data.clear();
    for (auto i = 0; i < n; i++)
    {
        data.push_back({(*new_belief)[i].r.get_x(), (*new_belief)[i].r.get_y()});
    }
    figure.series("Resampled Particles").set(data).type(cvplot::Dots).color(cvplot::Blue);

    //Draw particles in green
    data.clear();
    for (auto i = 0; i < n; i++)
    {
        data.push_back({(*belief)[i].r.get_x(), (*belief)[i].r.get_y()});
    }
    figure.series("Particles").set(data).type(cvplot::Dots).color(cvplot::Green);

    //Draw landmarks in red
    data.clear();
    for (int i = 0; i < map.size(); i ++)
    {
        data.push_back({map[i].x, map[i].y});
    }
    figure.series("Landmarks").set(data).type(cvplot::Dots).color(cvplot::Red);

    //Show the plot
    cvplot::figure(name).show();

}