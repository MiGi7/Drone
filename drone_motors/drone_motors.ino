#include <ArduinoBLE.h>

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

    Drone() : motor1(P1_11), motor2(P1_12), motor3(P1_15), motor4(P1_13){}

    bool calibrate(){
    
      motor1.period_ms(10);
      motor2.period_ms(10);
      motor3.period_ms(10);
      motor4.period_ms(10); 

      delay(500);
      motor1.pulsewidth_ms(2);
      motor2.pulsewidth_ms(2);
      motor3.pulsewidth_ms(2);
      motor4.pulsewidth_ms(2);
      delay(3000);
      motor1.pulsewidth_ms(1);
      motor2.pulsewidth_ms(1);
      motor3.pulsewidth_ms(1);
      motor4.pulsewidth_ms(1);
      delay(1000);
    
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

double kp=2.5;//3.55
double ki=0.025;//0.003
double kd=0.00;//2.05

int throttle=1000;
float desired_angle = 0;

BLEService droneControl("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

// create switch characteristic and allow remote device to read and write
BLEByteCharacteristic droneStart("19B10010-E8F2-537E-4F6C-D104768A1215", BLERead | BLEWrite);
BLEByteCharacteristic droneMovement("19B10010-E8F2-537E-4F6C-D104768A1216", BLERead | BLEWrite);
BLEByteCharacteristic droneMovementSlow("19B10010-E8F2-537E-4F6C-D104768A1217", BLERead | BLEWrite);
BLEByteCharacteristic droneStop("19B10010-E8F2-537E-4F6C-D104768A1218", BLERead | BLEWrite);


Drone myDrone;

  //usb

//a     c
// \   /
//  \ /
//  / \
// /   \
//d     b

void setup(){
  Serial.begin(9600);
 // while (!Serial);

    // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  
  BLE.setLocalName("Drone Controls");
  BLE.setAdvertisedService(droneControl);

  // add the characteristics to the service
  droneControl.addCharacteristic(droneStart);
  droneControl.addCharacteristic(droneMovement);
  droneControl.addCharacteristic(droneMovementSlow);
  droneControl.addCharacteristic(droneStop);

  // add the service
  BLE.addService(droneControl);

  droneStart.writeValue(1);  //needs to be 1 for flight
  droneMovement.writeValue(0);
  droneMovementSlow.writeValue(0);
  droneStop.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("Waiting for start");
  while(droneStart.value()){
    BLE.poll();
  }
  Serial.println("Arming motors");
  current_time = millis();
  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  myDrone.calibrate();
  delay(3000);

}

void loop(){
  BLE.poll();
  if (droneMovement.value()){
    throttle = throttle + 30;
    droneMovement.writeValue(0);
  }

  if (droneMovementSlow.value()){
    throttle = throttle - 20;
    droneMovementSlow.writeValue(0);
  }
  if(droneStop.value()){
    Serial.println("Drone has been stopped!");
    myDrone.motorSpeedAll(0);
    while(droneStop.value()){
      BLE.poll();
    }
  }

    timePrev = current_time;  // the previous time is stored before the actual time read
    current_time = millis();  // actual time read
    elapsedTime = (current_time - timePrev) / 1000; 

   if(IMU.accelerationAvailable()){
    IMU.readAcceleration(raw_accel_x, raw_accel_y, raw_accel_z);
  }

  //X angle
  accel_x = atan((raw_accel_y)/sqrt(pow((raw_accel_x),2) + pow((raw_accel_z),2)))*rad_to_deg;

  accel_y = atan(-1*(raw_accel_x)/sqrt(pow((raw_accel_y),2) + pow((raw_accel_z),2)))*rad_to_deg;

  
  if (IMU.gyroscopeAvailable()){
     IMU.readGyroscope(raw_gyro_x, raw_gyro_y, raw_gyro_z);
  }

  gyro_x = raw_gyro_x;
  gyro_y = raw_gyro_y;

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


if (PIDx > 100){
  PIDx = 100;
}

if (PIDx < -100){
  PIDx = -100;
}

if (PIDy > 100){
  PIDy = 100;
}

if (PIDy < -100){
  PIDy = -100;
}

int motora, motorb, motorc, motord;
//PIDx = PIDx / 3;
//PIDy = PIDy / 3;


if (throttle >= 1000){
  motora  = throttle - PIDx;
  motord = throttle - PIDx;
  
  motorc  = throttle + PIDx;
  motorb = throttle + PIDx;
  
  motora = motora - PIDy;
  motorb = motorb - PIDy;
  
  motorc = motorc + PIDy;
  motord = motord + PIDy;  
} else {
  motora  = throttle;
  motord = throttle;
  
  motorc  = throttle;
  motorb = throttle;
  }


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

if (motora > 1500){
  motora = 1500;
}

if (motorb > 1500){
  motorb = 1500;
}

if (motorc > 1500){
  motorc = 1500;
}

if (motord > 1500){
  motord = 1500;
}

Serial.println(motora);
Serial.println(motorb);
Serial.println(motorc);
Serial.println(motord);

myDrone.motorSpeed(motora, motorb, motorc, motord);

}
