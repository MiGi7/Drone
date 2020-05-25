#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#include "Drone.h"
#include "PID.h"

#define LEDR        (22u)
#define LEDG        (23u)
#define LEDB        (24u)

void ledLight(int color){
  //blue
  if (color == 1){
    digitalWrite(LEDG, HIGH); 
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDR, HIGH); 
    //green
  } else if (color == 2){
    digitalWrite(LEDG, LOW); 
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDR, HIGH);
    //red
  } else {
    digitalWrite(LEDG, HIGH); 
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDR, LOW);
  }
}


void getIMUData(float &a_x, float &a_y,float &a_z,float &g_x,float &g_y,float &g_z){
  if(IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
    IMU.readAcceleration(a_x, a_y, a_z);
    IMU.readGyroscope(g_x, g_y, g_z);
  }
}

float raw_gyro_x, raw_gyro_y, raw_gyro_z, raw_accel_x, raw_accel_y, raw_accel_z;

unsigned long loop_time;


double kp = 1;//3.55
double ki = 0.01;//0.003
double kd = 0;//2.05

PID* pitch = new PID(kp, ki, kd);
PID* roll = new PID(kp, ki, kd);

Drone myDrone;


BLEService droneControl("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

// create switch characteristic and allow remote device to read and write
BLEByteCharacteristic droneStart("19B10010-E8F2-537E-4F6C-D104768A1215", BLERead | BLEWrite);
BLEByteCharacteristic droneMovement("19B10010-E8F2-537E-4F6C-D104768A1216", BLERead | BLEWrite);
BLEByteCharacteristic droneMovementSlow("19B10010-E8F2-537E-4F6C-D104768A1217", BLERead | BLEWrite);
BLEByteCharacteristic droneStop("19B10010-E8F2-537E-4F6C-D104768A1218", BLERead | BLEWrite);



  //usb

//a     c
// \   /
//  \ /
//  / \
// /   \
//d     b



void setup(){
  myDrone.attachPitch(pitch);
  myDrone.attachRoll(roll);

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  
  ledLight(3);
  ledLight(1);
  ledLight(3);

  Serial.begin(115200);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  //while (!Serial);
  Serial.println("Serial Port Started");
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
  ledLight(3);
  // start advertising
  BLE.advertise();

  ledLight(1);
  
  Serial.println("Waiting for start");
  while(droneStart.value()){
    BLE.poll();
  }
  ledLight(2);
  Serial.println("Arming motors");
 
  myDrone.armMotors();
  delay(3000);

  loop_time = micros();
}

void loop(){
  getIMUData(raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z);
  //Serial.println(myDrone.makeCorrection(micros() - loop_time, raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z));
  myDrone.motorSpeedAll(1050);
  loop_time = micros();

  if(pitch->totalAngle(raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, micros() - loop_time) > 25){
    myDrone.motorSpeedAll(1000);
    while(1);
  }
}
