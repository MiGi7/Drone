#include <Servo.h>


#ifndef DRONE_H
#define DRONE_H

class Drone {
  // Commands a drone with 4 motors using servo and calibrate the ESC
  public:

    Servo motor1;
    Servo motor2;
    Servo motor3;
    Servo motor4;

    Drone();

    bool connectMotors(int motor_pin1, int motor_pin2, int motor_pin3, int motor_pin4);
    bool calibrate();

    //bool motorSpeed(int id, int motor_speed);
    bool motorSpeedAll(int motor_speed);

    //void motorSpeedPercent();

};

#endif
