#include "MeAuriga.h"

MeEncoderOnBoard motorR(SLOT_1);
MeEncoderOnBoard motorL(SLOT_2);


MeLineFollower lineSensor(PORT_8);
uint8_t currReading;
uint8_t prevReading = -1;

void setup() {
  //Serial.begin(9600);
}

void loop() {
  currReading = lineSensor.readSensors();
  if (currReading != prevReading) { 
    lineFollow(currReading);
  }
  prevReading = currReading;
  
  delay(50);
}


void lineFollow(uint8_t reading) {
    switch(reading) {
    case 0:
      //Serial.println("Both inside");
      driveFwd(250);
      break;
    case 1:
      //Serial.println("S2(R) Out, S1(L) In");
      turnLeft(150);
      break;
    case 2:
      //Serial.println("S2(R) In, S1(L) Out");
      turnRight(150);
      break;
    case 3:
      turnRight(250);
      //Serial.println("Both outside");
      break;
  }
}


void driveFwd(int16_t pwm) {
  motorR.setMotorPwm(-pwm);
  motorL.setMotorPwm(pwm);
}

void driveBwd(int16_t pwm) {
  motorR.setMotorPwm(pwm);
  motorL.setMotorPwm(-pwm);
}

void turnLeft(int16_t pwm) {
  motorR.setMotorPwm(-pwm);
  motorL.setMotorPwm(-pwm);
}

void turnRight(int16_t pwm) {
  motorR.setMotorPwm(pwm);
  motorL.setMotorPwm(pwm);
}
