#include <Servo.h> 

class DroneController {
  
};

void armMotors(Servo motor_1, Servo motor_2, Servo motor_3, Servo motor_4){
   int pos = 0;
     // attaches the servo on pin 9 to the servo object 
for(pos = 0; pos < 179; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    motor_1.write(pos);
    motor_2.write(pos);
    motor_3.write(pos);
    motor_4.write(pos);
    // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 179; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    motor_1.write(pos);
    motor_2.write(pos);
    motor_3.write(pos);
    motor_4.write(pos);// tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
}

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;  
int val = 40;
 
void setup() { 
  Serial.begin(9600);
    myservo1.attach(3);
    myservo2.attach(5);
    myservo3.attach(6);
    myservo4.attach(11);
    armMotors(myservo1, myservo2, myservo3, myservo4);
} 
 
void loop() 
{
    val = map(analogRead(A0),0,1023,40,210); //42 initiates motor spin
    myservo1.write(val); //42 base
    delay(15);
    myservo2.write(val + 26); //68 base 
    delay(15);
    myservo3.write(val+ 20); //62 base 
    delay(15);
    myservo4.write(val); //42 base
    delay(15);
    Serial.println(val);
} 
