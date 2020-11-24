#include "MeAuriga.h"

#define SPEED_SLOW   150
#define SPEED_NORMAL 200
#define SPEED_HIGH   255

MeEncoderOnBoard motorR(SLOT_1);
MeEncoderOnBoard motorL(SLOT_2);

MeUltrasonicSensor ultra(PORT_8);
double distance;

MePort port(PORT_9);
Servo servo;  // create servo object to control a servo
int16_t servoPin =  port.pin1();//attaches the servo on PORT_3 SLOT1 to the servo object
int16_t servoPos = 90;

char command;

void setup()
{
  Serial.begin(9600);
  servo.attach(servoPin);  // attaches the servo on servopin1
}

void loop()
{   
  if (Serial.available() > 0) {
    command = Serial.read();
  }
  
  if (command == 'w') {
    Serial.println("Forward");
    driveFwd(SPEED_NORMAL);
  }
  else if (command == 's') {
    Serial.println("Backward");
    driveBwd(SPEED_NORMAL);
  }
  else if (command == 'a') {
    Serial.println("Left");
    turnLeft(SPEED_NORMAL);
  }
  else if (command == 'd') {
    Serial.println("Right");
    turnRight(SPEED_NORMAL);
  }
  else if (command == 'q') {
    Serial.println("Servo++");
    servoPos += 5;
    if (servoPos >= 180) 
      servoPos = 180;
    servo.write(servoPos);
  }
  else if (command == 'e') {
    Serial.println("Servo++");
    servoPos -= 5;
    if (servoPos <= 0) 
      servoPos = 0;
    servo.write(servoPos);
  }
  else if (command == 'r') {
    servoPos = 90;
    servo.write(servoPos);
  }
  else {
    Serial.println("Stop");
    driveFwd(0);
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
