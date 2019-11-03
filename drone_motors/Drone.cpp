#include "Drone.h"
#include "Arduino.h"

Drone::Drone(){
 motor1;
 motor2;
 motor3;
 motor4;
 Serial.begin(9600);
}

bool Drone::calibrate(){
  this->motor1.writeMicroseconds(2000);
  this->motor2.writeMicroseconds(2000);
  this->motor3.writeMicroseconds(2000);
  this->motor4.writeMicroseconds(2000);
  delay(1000);
  this->motor1.writeMicroseconds(1000);
  this->motor2.writeMicroseconds(1000);
  this->motor3.writeMicroseconds(1000);
  this->motor4.writeMicroseconds(1000);
  return true;
}

bool Drone::connectMotors(int motor_pin1, int motor_pin2, int motor_pin3, int motor_pin4){
  this->motor1.attach(motor_pin1);
  this->motor2.attach(motor_pin2);
  this->motor3.attach(motor_pin3);
  this->motor4.attach(motor_pin4);
  this->calibrate();
}

bool Drone::motorSpeedAll(int motor_speed){
  if(motor_speed < 999 || motor_speed >= 1200){
    return false;
  }
  this->motor1.writeMicroseconds(motor_speed);
  this->motor2.writeMicroseconds(motor_speed);
  this->motor3.writeMicroseconds(motor_speed);
  this->motor4.writeMicroseconds(motor_speed);
}
