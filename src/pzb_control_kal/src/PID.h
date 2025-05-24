#ifndef PID_h
    #define PID_h

#include <math.h>
#include <stdlib.h>

class PID
{
    public:
        float sample_time_;
        
        float error_;
        float prev_error_;

        float k_p_;
        float k_i_;
        float k_d_;

        float U_MIN_;
        float U_MAX_;
        
        float u_;
        float chi1_d_;

        // May even be usefull to create another constructor without u_max, as when PID is FBLinearized, the FBLin
        // base class already saturates the signals
        PID();
        PID(float sample_time, float k_p, float k_i, float k_d, float u_max, float u_min);
        
        void updateReferences(float chi1_d);
        void calculateManipulation(float chi1);
        
        // Saturate manipulation function is intended to be used in applications where a FBLin PID is not required,
        // as FBLin base classes already saturate the control signals
        void saturateManipulation(float chi1);
};

#endif