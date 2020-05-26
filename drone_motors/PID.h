#ifndef PID_h
#define PID_h

class PID {
    protected:
        float desired_angle = 0;

        const double proportional;
        const double integral;
        const double derivative;

        float p_adjust = 0;
        float i_adjust = 0;
        float d_adjust = 0;

        double previous_angle = 0;
        float previous_time = 0;
        float previous_error = 0;


    public:
    
    PID(double proportional, double integral, double derivative);

    float totalAngle(float accel_1, float accel_2, float accel_3, float gyro_data, unsigned long time_elapsed);
    
    float adjustment(float accel_1, float accel_2, float accel_3, float gyro_data, unsigned long time_elapsed);
    
};

#endif