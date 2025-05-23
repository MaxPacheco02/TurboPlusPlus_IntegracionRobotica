#include "PID.h"

PID::PID()
{
    sample_time_    = 0;
    k_p_            = 0;
    k_i_            = 0;
    k_d_            = 0;

    error_          = 0;
    prev_error_     = 0;
    chi1_d_         = 0;
    u_              = 0;

    U_MAX_ = 0;
    U_MIN_ = 0;

}

PID::PID(float sample_time, float k_p, float k_i, float k_d, float u_max, float u_min)
{
    sample_time_    = sample_time;
    k_p_            = k_p;
    k_i_            = k_i;
    k_d_            = k_d;

    error_          = 0;
    prev_error_     = 0;
    chi1_d_         = 0;
    u_              = 0;

    U_MAX_ = u_max;
    U_MIN_ = u_min;
}

void PID::updateReferences(float chi1_d)
{
    chi1_d_ = chi1_d;
}

void PID::calculateManipulation(float chi1)
{
    float error_d;
    float error_i;
    float u;

    prev_error_    = error_;
    error_         = chi1_d_ - chi1;

    error_d = (error_ - prev_error_) / sample_time_;
    error_i = ((error_ + prev_error_) / 2 * sample_time_) + error_;

    u  = k_p_ * error_ + k_i_ * error_i + k_d_ * error_d;
                                                               
    if(!isnan(u) || u != 0.0)
        u_ = u;
}

void PID::saturateManipulation(float chi1)
{
    calculateManipulation(chi1);
    u_ = abs(u_) > U_MAX_ ? u_ / abs(u_) * U_MAX_ : u_;
    u_ = u_ < U_MIN_ ? U_MIN_ : u_;
}