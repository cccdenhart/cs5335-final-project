
#include "MeAuriga.h"

MeEncoderOnBoard motorR(SLOT_1);
MeEncoderOnBoard motorL(SLOT_2);

MeLineFollower lineSensorFront(PORT_10);

MeUltrasonicSensor ultra(PORT_9);
double distance;

MeRGBLed leds(0);
#define LEDNUM  12
#define LEDPORT 44
#define BLUE    0, 0, 255
#define RED     255, 0, 0
#define OFF     0, 0, 0
short int col;

MeBuzzer siren;
#define BUZZER_PORT 45

short int STATE;
unsigned long startTime;
unsigned long endTime;

void setup() {
  leds.setpin(LEDPORT);
  leds.setNumber(LEDNUM);
  leds.setColor(OFF);
  leds.show();
  
  siren.setpin(BUZZER_PORT);
  siren.noTone();
  
  STATE = 0;
  col = 0;
  Serial.begin(9600);
}


void loop() {
  checkState();

  if (STATE == 0) {
    driveFwd(0);
    siren.noTone();
    leds.setColor(OFF);
    leds.show();
  }
  
  if (STATE == 1) {
    lineFollow();
    sirenAndLights();
  } 

  delay(50);
}


void checkState() {
  distance = ultra.distanceCm();

  if (STATE == 0) {
    if (distance <= 30) {
      STATE = 1;
      startTime = millis();
    }
  } else {
    endTime = millis();
    if (endTime - startTime >= 10000) {
      STATE = 0;      
    }
  }
}


void lineFollow() {
  switch(lineSensorFront.readSensors()) {
    case 0:
      //Serial.println("Both inside");
      driveFwd(200);
      break;
    case 1:
      //Serial.println("S2(R) Out, S1(L) In");
      correctLeft(150);
      break;
    case 2:
      //Serial.println("S2(R) In, S1(L) Out");
      correctRight(150);
      break;
    case 3:
      swirl(150);
      //Serial.println("Both outside");
      break;
  }
}


void sirenAndLights() {
  if (col == 0) {
    leds.setColor(RED);
    leds.show();
    siren.tone(523, 100);
    col++;
  } else {
    leds.setColor(BLUE);
    leds.show();
    siren.tone(370, 100);
    col--;
  }
}


void driveFwd(int16_t pwm) {
  motorR.setMotorPwm(-pwm);
  motorL.setMotorPwm(pwm);
}

void correctLeft(int16_t pwm) {
  motorR.setMotorPwm(-pwm);
  motorL.setMotorPwm(pwm - pwm/3);
}

void correctRight(int16_t pwm) {
  motorR.setMotorPwm(-pwm + pwm/3);
  motorL.setMotorPwm(pwm);
}

void swirl(int16_t pwm) {
  motorR.setMotorPwm(pwm);
  motorL.setMotorPwm(pwm);
}
