#include "Drone.h"

#define motor_a P1_11
#define motor_b P1_12
#define motor_c P1_15
#define motor_d P1_13

Drone::Drone() : motor1(motor_a), motor2(motor_b), motor3(motor_c), motor4(motor_d){
    motor1.period_ms(10);
    motor2.period_ms(10);
    motor3.period_ms(10);
    motor4.period_ms(10); 
}

void Drone::calibrate(){

    motor1.pulsewidth_ms(2);
    motor2.pulsewidth_ms(2);
    motor3.pulsewidth_ms(2);
    motor4.pulsewidth_ms(2);
    wait(5);

    motor1.pulsewidth_ms(1);
    motor2.pulsewidth_ms(1);
    motor3.pulsewidth_ms(1);
    motor4.pulsewidth_ms(1);
    wait(5);
     
    
}

void Drone::armMotors(){
    motor1.pulsewidth_us(1100);
    motor2.pulsewidth_us(1100);
    motor3.pulsewidth_us(1100);
    motor4.pulsewidth_us(1100);
    wait(2);

    motor1.pulsewidth_ms(1);
    motor2.pulsewidth_ms(1);
    motor3.pulsewidth_ms(1);
    motor4.pulsewidth_ms(1);
    wait(2);
    motorSpeedAll(1150);
}

void Drone::motorSpeedAll(int speed_percent){
    motor1.pulsewidth_us(speed_percent);
    motor2.pulsewidth_us(speed_percent);
    motor3.pulsewidth_us(speed_percent);
    motor4.pulsewidth_us(speed_percent);
}

void Drone::motorSpeed(int a, int b, int c, int d){
    motor1.pulsewidth_us(a);
    motor2.pulsewidth_us(b);
    motor3.pulsewidth_us(c);
    motor4.pulsewidth_us(d);
}

void Drone::attachPitch(PID* pid){
    this->pitch = pid;
}

void Drone::attachRoll(PID* pid){
    this->roll = pid;
}

void Drone::attachYaw(PID* pid){
    this->yaw = pid;
}

void Drone::setBaseSpeed(int change){
    base_motor_speed = base_motor_speed + change;
}

int Drone::makeCorrection(unsigned long time_elapsed, float accel_x, float accel_y, float accel_z, 
    float gyro_x, float gyro_y, float gyro_z){
    int motorSpeed_a, motorSpeed_b, motorSpeed_c, motorSpeed_d;
    if(base_motor_speed > 1120){
        int pitch_adjustment = pitch->adjustment(accel_x, accel_y, accel_z, gyro_x, time_elapsed);
        int roll_adjustment = roll->adjustment(accel_y, accel_x, accel_z, gyro_y, time_elapsed);
        motorSpeed_a = base_motor_speed - pitch_adjustment - roll_adjustment;
        motorSpeed_b = base_motor_speed + pitch_adjustment + roll_adjustment;
        motorSpeed_c = base_motor_speed - pitch_adjustment + roll_adjustment;
        motorSpeed_d = base_motor_speed + pitch_adjustment - roll_adjustment;
        motorSpeed(motorSpeed_a, motorSpeed_b, motorSpeed_c, motorSpeed_d);
    } else {
        motorSpeed(base_motor_speed, base_motor_speed, base_motor_speed, base_motor_speed);
    }
    return time_elapsed;
}


