
#include "Drone.h"

Drone::Drone(){

}

bool Drone::calibrate(){

  motor1.period_ms(20);
  motor2.period_ms(20);
  motor3.period_ms(20);
  motor4.period_ms(20);

  motor1.pulsewidth_ms(2);
  motor2.pulsewidth_ms(2);
  motor3.pulsewidth_ms(2);
  motor4.pulsewidth_ms(2);


  motor1.pulsewidth_ms(1);
  motor2.pulsewidth_ms(1);
  motor3.pulsewidth_ms(1);
  motor4.pulsewidth_ms(1);

  return true;
}

bool Drone::motorSpeedAll(int motor_speed_percent){
  int speed_percent = map(motor_speed_percent, 0, 100, 1000, 2000);
  motor1.pulsewidth_us(speed_percent);
  motor2.pulsewidth_us(speed_percent);
  motor3.pulsewidth_us(speed_percent);
  motor4.pulsewidth_us(speed_percent);
  return true;
}
