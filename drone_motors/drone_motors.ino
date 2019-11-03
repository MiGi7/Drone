#include "Drone.h"

Drone myDrone;

void setup(){
  myDrone.connectMotors(5,6,10,11);
}

void loop(){
 delay(5000);
 myDrone.motorSpeedAll(1100);
}
