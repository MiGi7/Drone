

#include "mbed.h"

mbed::PwmOut motor1(P1_11); //Pin D2
mbed::PwmOut motor2(P1_12); //Pin D3
mbed::PwmOut motor3(P1_15); //Pin D4
mbed::PwmOut motor4(P1_13); //Pin D5

#ifndef DRONE_H
#define DRONE_H


class Drone {
  // Commands a drone with 4 motors using servo and calibrate the ESC. Sets
  //motor pins at 2, 3, 4 and 5
  public:

      mbed::PwmOut motor1; //Pin D2
      mbed::PwmOut motor2; //Pin D3
      mbed::PwmOut motor3; //Pin D4
      mbed::PwmOut motor4; //Pin D5

    Drone();

    bool calibrate();

    //bool motorSpeed(int id, int motor_speed);
    bool motorSpeedAll(int motor_speed);

    //void motorSpeedPercent();

};

#endif
