#include "PID.h"
#include <math.h>

#define rad_to_deg 180/M_PI

PID::PID(double proportional, double integral, double derivative) : 
proportional{proportional}, integral{integral}, derivative{derivative} {}


float PID::totalAngle(float accel_1, float accel_2, float accel_3, float gyro_data, unsigned long time_elapsed){
    float total_angle = (0.98 * (previous_angle + (gyro_data * time_elapsed * 0.000001))) + 
    (0.02 * (atan(accel_1/(sqrt(pow(accel_2, 2) + pow(accel_3, 2)))) * rad_to_deg));
    previous_angle = total_angle;
    return total_angle;
}

float PID::adjustment(float accel_1, float accel_2, float accel_3, float gyro_data, unsigned long time_elapsed){
    double current_angle = totalAngle(accel_1, accel_2, accel_3, gyro_data, time_elapsed);
    float error = current_angle - desired_angle;
    p_adjust = proportional * error;
    if (error < 3 || error > -3) {
        i_adjust = i_adjust + (integral * error);
    }
    i_adjust = integral;
    d_adjust = derivative * ((error - previous_error)/time_elapsed) * 0.000001;
    previous_error = error;
    float adjustFloat = p_adjust + i_adjust + d_adjust;
    int adjustmentInt = (adjustFloat < 500 || adjustFloat > -500) ? (int)adjustFloat : 0;
    return adjustmentInt;
}


