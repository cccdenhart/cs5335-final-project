
#include "MeAuriga.h"

MeEncoderOnBoard motorR(SLOT_1);
MeEncoderOnBoard motorL(SLOT_2);

MeLineFollower lineSensorFront(PORT_10);
MeLineFollower lineSensorBack(PORT_8);



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
short int DIRECTION;
unsigned long startTime;
unsigned long endTime = 5000;
#define MIN_DURATION 5000
#define MAX_DURATION 20000


void setup() {
  leds.setpin(LEDPORT);
  leds.setNumber(LEDNUM);
  leds.setColor(OFF);
  leds.show();

  siren.setpin(BUZZER_PORT);
  siren.noTone();

  randomSeed(analogRead(0));
  DIRECTION = 0;
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
  } else {
    if (DIRECTION == 0) {
      lineFollow(0);
      sirenAndLights();
    } else {
      lineFollow(1);
      sirenAndLights();
    }
  }

  //printState();

  delay(50);
}

void checkState() {
  if (STATE == 0) {
    if (millis() - startTime >= endTime) {
      STATE = 1;
      getDirection();
      startTime = millis();
      endTime = random(MIN_DURATION, MAX_DURATION);
    }
  } else {
    if (millis() - startTime >= endTime) {
      STATE = 0;
      startTime = millis();
      endTime = random(MIN_DURATION, MAX_DURATION);
    }
  }
}


void getDirection() {
  DIRECTION = random(2);
}


void lineFollow(short int DIR) {
  if (DIR == 0) {
    switch (lineSensorFront.readSensors()) {
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
  } else {
    switch (lineSensorBack.readSensors()) {
      case 0:
        //Serial.println("Both inside");
        driveFwd(-200);
        break;
      case 1:
        //Serial.println("S2(R) Out, S1(L) In");
        correctLeft(-150);
        break;
      case 2:
        //Serial.println("S2(R) In, S1(L) Out");
        correctRight(-150);
        break;
      case 3:
        swirl(-150);
        //Serial.println("Both outside");
        break;
    }
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
  motorL.setMotorPwm(pwm - pwm / 3);
}

void correctRight(int16_t pwm) {
  motorR.setMotorPwm(-pwm + pwm / 3);
  motorL.setMotorPwm(pwm);
}

void swirl(int16_t pwm) {
  motorR.setMotorPwm(pwm);
  motorL.setMotorPwm(pwm);
}


void printState() {
  Serial.print(STATE);
  Serial.print("\t");
  Serial.print((millis() - startTime) / 1000);
  Serial.print("\t");
  Serial.print(endTime);
  Serial.print("\t");
  Serial.println(DIRECTION);
}
