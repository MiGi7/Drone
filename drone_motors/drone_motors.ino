#include <Arduino_LSM9DS1.h>

#include "mbed.h"

class Drone {
  // Commands a drone with 4 motors using servo and calibrate the ESC. Sets
  //motor pins at 2, 3, 4 and 5
  public:

    mbed::PwmOut motor1; //Pin D2
    mbed::PwmOut motor2; //Pin D3
    mbed::PwmOut motor3; //Pin D4
    mbed::PwmOut motor4; //Pin D5

    Drone() : motor1 (P1_11), motor2 (P1_12), motor3(P1_15), motor4(P1_13){}

    bool calibrate(){
    
      motor1.period_ms(20);
      motor2.period_ms(20);
      motor3.period_ms(20);
      motor4.period_ms(20);
    
      motor1.pulsewidth_ms(2);
      motor2.pulsewidth_ms(2);
      motor3.pulsewidth_ms(2);
      motor4.pulsewidth_ms(2);
      delay(2000);
      motor1.pulsewidth_ms(1);
      motor2.pulsewidth_ms(1);
      motor3.pulsewidth_ms(1);
      motor4.pulsewidth_ms(1);
      delay(3000);
    
      return true;
    }

    //bool motorSpeed(int id, int motor_speed);
    bool motorSpeedAll(int motor_speed_percent){
      int speed_percent = map(motor_speed_percent, 0, 100, 1000, 2000);
      motor1.pulsewidth_us(speed_percent);
      motor2.pulsewidth_us(speed_percent);
      motor3.pulsewidth_us(speed_percent);
      motor4.pulsewidth_us(speed_percent);
      return true;
    }

     bool motorSpeed(int a, int b, int c, int d){
      motor1.pulsewidth_us(a);
      motor2.pulsewidth_us(b);
      motor3.pulsewidth_us(c);
      motor4.pulsewidth_us(d);
      return true;
    }

    //void motorSpeedPercent();

};

float raw_gyro_x, raw_gyro_y, raw_gyro_z, raw_accel_x, raw_accel_y, raw_accel_z;

float gyro_x, gyro_y, accel_x, accel_y;

float total_angle_x, total_angle_y;

float elapsedTime, current_time, timePrev;

const float rad_to_deg = 180/3.141592654;

float PIDx, PIDy ,current_error_x, current_error_y, previous_error_x, previous_error_y;
float pid_p_x=0;
float pid_i_x=0;
float pid_d_x=0;
float pid_p_y=0;
float pid_i_y=0;
float pid_d_y=0;

double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05

int throttle=1000;
float desired_angle = 0;

Drone myDrone;

void setup(){
  current_time = millis();
  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  myDrone.calibrate();
  //myDrone.motorSpeedAll(5);
  delay(3000);

}

void loop(){

    timePrev = current_time;  // the previous time is stored before the actual time read
    current_time = millis();  // actual time read
    elapsedTime = (current_time - timePrev) / 1000; 

   if(IMU.accelerationAvailable()){
    IMU.readAcceleration(raw_accel_x, raw_accel_y, raw_accel_z);
  }

  //X angle
  gyro_x = atan((raw_accel_y)/sqrt(pow((raw_accel_x),2) + pow((raw_accel_z),2)))*rad_to_deg;

  accel_y = atan(-1*(raw_accel_x)/sqrt(pow((raw_accel_y),2) + pow((raw_accel_z),2)))*rad_to_deg;

  
  if (IMU.gyroscopeAvailable()){
     IMU.readGyroscope(raw_gyro_x, raw_gyro_y, raw_gyro_z);
  }

  gyro_x = raw_gyro_x;
  gyro_y =raw_gyro_y;

    /*---X axis angle---*/
   total_angle_x = 0.98 *(total_angle_x + gyro_x*elapsedTime) + 0.02*accel_x;
   /*---Y axis angle---*/
   total_angle_y = 0.98 *(total_angle_y + gyro_y*elapsedTime) + 0.02*accel_y;

   current_error_x = total_angle_x - desired_angle;
   current_error_y = total_angle_y - desired_angle;

  pid_p_x = kp * current_error_x;
  pid_p_y = kp * current_error_y;

  if(-3 < current_error_x < 3)
{
  pid_i_x = pid_i_x+(ki*current_error_x);  
}

 if(-3 < current_error_y < 3)
{
  pid_i_y = pid_i_y+(ki*current_error_y);  
}

 pid_d_x = kd*((current_error_x - previous_error_x)/elapsedTime);
 pid_d_y = kd*((current_error_y - previous_error_y)/elapsedTime);

 PIDx = pid_p_x + pid_i_x + pid_d_x;
 PIDy = pid_p_y + pid_i_y + pid_d_y;

 if(PIDx < -1000)
{
  PIDx=-1000;
}
if(PIDx > 1000)
{
  PIDx=1000;
}

if(PIDy < -1000)
{
  PIDy=-1000;
}
if(PIDy > 1000)
{
  PIDy=1000;
}


int motora  = throttle + PIDx;
int motorb = throttle - PIDx;

int motorc  = throttle + PIDy;
int motord = throttle - PIDy;

if (motora < 1000){
  motora = 1000;
}

if (motorb < 1000){
  motorb = 1000;
}

if (motorc < 1000){
  motorc = 1000;
}

if (motord < 1000){
  motord = 1000;
}

Serial.println(motora);
Serial.println(motorb);
Serial.println(motorc);
Serial.println(motord);

myDrone.motorSpeed(motora, motorb, motorc, motord);


  //IMU.readAcceleration(raw_accel_x, raw_accel_y, raw_accel_z);
  //int input_speed = map(analogRead(A0), 0, 1023, 0, 100);
  //myDrone.motorSpeedAll(input_speed);
  //Serial.print("Current Motor speed: ");
  //Serial.print(input_speed);
  //Serial.println("%");

}