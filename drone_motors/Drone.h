#include "mbed.h"
#include "PID.h"

#ifndef Drone_h
#define Drone_h


  //usb

//d     b
// \   /
//  \ /
//  / \
// /   \
//a     c

//motor a and b are correct but not calibrated

class Drone {
  // Commands a drone with 4 motors using servo and calibrate the ESC. Sets
  //motor pins at 2, 3, 4 and 5
  private:


  public:
    int base_motor_speed = 1150;

    mbed::PwmOut motor1; //Pin D2
    mbed::PwmOut motor2; //Pin D3
    mbed::PwmOut motor3; //Pin D4
    mbed::PwmOut motor4; //Pin D5

    PID* pitch;
    PID* roll;
    PID* yaw;

    Drone();

    void armMotors();

    void calibrate();

    void motorSpeedAll(int motor_speed_percent);

    void motorSpeed(int a, int b, int c, int d);

    void setBaseSpeed(int change);

    void attachPitch(PID* pid);

    void attachRoll(PID* pid);

    void attachYaw(PID* pid);

    int makeCorrection(unsigned long time_elapsed, float accel_x, float accel_y, float accel_z, 
      float gyro_x, float gyro_y, float gyro_z);
};

#endif 