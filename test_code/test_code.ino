#include <Servo.h> 

Servo myservo;

void setup() 
{ 
  myservo.attach(5);
  myservo.writeMicroseconds(2000);  // set servo to mid-point
  delay(1000);
  myservo.writeMicroseconds(1000);
  delay(1000);
  myservo.writeMicroseconds(1100);
} 

void loop() {} 
