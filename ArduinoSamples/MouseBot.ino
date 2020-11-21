#include "MeAuriga.h"

MeEncoderOnBoard motorR(SLOT_1);
MeEncoderOnBoard motorL(SLOT_2);

MeUltrasonicSensor ultra(PORT_8);
double distanceL;
double distanceR;
double distanceF;

MePort port(PORT_9);
Servo servo;  // create servo object to control a servo
int16_t servoPin =  port.pin1();//attaches the servo on PORT_3 SLOT1 to the servo object

void setup()
{
  servo.attach(servoPin);  // attaches the servo on servopin1
}

void loop()
{   
  servo.write(0); // set servo to the Right
  delay(100);
  distanceR = ultra.distanceCm();

  if (distanceR < 15) {   // getting too close to the wall, correct Left
    correctLeft(200);
  } else if (distanceR > 60) {      
      turnRight(200);
  } else if (distanceR > 20) {
      correctRight(200);
  } else {
      driveFwd(200);
  }
  
  delay(100);
}


void driveFwd(int16_t pwm) {
  motorR.setMotorPwm(-pwm);
  motorL.setMotorPwm(pwm);
}

void driveBwd(int16_t pwm) {
  motorR.setMotorPwm(pwm);
  motorL.setMotorPwm(-pwm);
}

void correctLeft(int16_t pwm) {
  motorR.setMotorPwm(-pwm);
  motorL.setMotorPwm(pwm - pwm/4);
}

void correctRight(int16_t pwm) {
  motorR.setMotorPwm(-pwm + pwm/4);
  motorL.setMotorPwm(pwm);
}

void turnLeft(int16_t pwm) {
  motorR.setMotorPwm(-pwm);
  motorL.setMotorPwm(pwm/2);
}

void turnRight(int16_t pwm) {
  motorR.setMotorPwm(-pwm/2);
  motorL.setMotorPwm(pwm);
}

void swirl(int16_t pwm) {
  motorR.setMotorPwm(pwm);
  motorL.setMotorPwm(pwm);
}
