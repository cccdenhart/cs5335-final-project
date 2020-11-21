#include "MeAuriga.h"

MeEncoderOnBoard motorR(SLOT_1);
MeEncoderOnBoard motorL(SLOT_2);

MeUltrasonicSensor ultra(PORT_8);
double distanceL;
double distanceR;
double distanceF;

MePort port(PORT_9);
Servo head;  // create servo object to control a servo
int16_t servoPin =  port.pin1();//attaches the servo on PORT_3 SLOT1 to the servo object


#define LPT 2 // scan loop counter

#define FAST_SPEED  250     //both sides of the motor speed
#define SPEED  120     //both sides of the motor speed
#define TURN_SPEED  200     //both sides of the motor speed
#define BACK_SPEED1  255     //back speed
#define BACK_SPEED2  90     //back speed

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30; //distance limit for obstacles in front
const int sidedistancelimit = 30; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;
const int turntime = 250; //Time the robot spends turning (miliseconds)
const int backtime = 300; //Time the robot spends turning (miliseconds)

int thereis;

void go_Advance(int16_t pwmL, int16_t pwmR) {
  motorR.setMotorPwm(-pwmR);
  motorL.setMotorPwm(pwmL);
}

void go_Back(int16_t pwmL, int16_t pwmR) {
  motorR.setMotorPwm(pwmR);
  motorL.setMotorPwm(-pwmL);
}

void correctLeft(int16_t pwm) {
  motorR.setMotorPwm(-pwm);
  motorL.setMotorPwm(pwm - pwm / 4);
}

void correctRight(int16_t pwm) {
  motorR.setMotorPwm(-pwm + pwm / 4);
  motorL.setMotorPwm(pwm);
}

void go_Left(int16_t pwmL, int16_t pwmR) {
  motorR.setMotorPwm(-pwmR);
  motorL.setMotorPwm(pwmL);
}

void go_Right(int16_t pwmL, int16_t pwmR) {
  motorR.setMotorPwm(-pwmR);
  motorL.setMotorPwm(pwmL);
}

void stop_Stop() {
  motorR.setMotorPwm(0);
  motorL.setMotorPwm(0);
}

void swirl(int16_t pwm = FAST_SPEED) {
  motorR.setMotorPwm(pwm);
  motorL.setMotorPwm(pwm);
}

/*detection of ultrasonic distance*/
int watch() {
  return round(ultra.distanceCm());
}

//Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval,
//leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
String watchsurrounding() {
  /*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
       for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
  */

  int obstacle_status = B100000;
  centerscanval = watch();
  if (centerscanval < distancelimit) {
    obstacle_status  = obstacle_status | B100;
  }
  
  head.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if (ldiagonalscanval < distancelimit) {
    obstacle_status  = obstacle_status | B1000;
  }
  head.write(170); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = watch();
  if (leftscanval < sidedistancelimit) {
    obstacle_status  = obstacle_status | B10000;
  }

  head.write(90); //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if (centerscanval < distancelimit) {
    obstacle_status  = obstacle_status | B100;
  }
  head.write(40);
  delay(100);
  rdiagonalscanval = watch();
  if (rdiagonalscanval < distancelimit) {
    obstacle_status  = obstacle_status | B10;
  }
  head.write(0);
  delay(100);
  rightscanval = watch();
  if (rightscanval < sidedistancelimit) {
    obstacle_status  = obstacle_status | 1;
  }
  head.write(90); //Finish looking around (look forward again)
  delay(300);
  String obstacle_str = String(obstacle_status, BIN);
  obstacle_str = obstacle_str.substring(1, 6);

  return obstacle_str; //return 5-character string standing for 5 direction obstacle status
}

void auto_avoidance() {

  ++numcycles;
  if (numcycles >= LPT) { //Watch if something is around every LPT loops while moving forward
    stop_Stop();
    String obstacle_sign = watchsurrounding(); // 5 digits of obstacle_sign binary value means the 5 direction obstacle status
    Serial.print("begin str=");
    Serial.println(obstacle_sign);
    if ( obstacle_sign == "10000") {
      Serial.println("SLIT right");
      go_Advance(FAST_SPEED, SPEED);
      delay(turntime);
    }
    else    if ( obstacle_sign == "00001"  ) {
      Serial.println("SLIT LEFT");
      go_Advance(SPEED, FAST_SPEED);

      delay(turntime);
    }
    else if ( obstacle_sign == "11100" || obstacle_sign == "01000" || obstacle_sign == "11000"  || obstacle_sign == "10100"  || obstacle_sign == "01100" || obstacle_sign == "00100"  || obstacle_sign == "01000" ) {
      Serial.println("hand right");
      go_Right(TURN_SPEED, TURN_SPEED);
      delay(turntime);
    }
    else if ( obstacle_sign == "00010" || obstacle_sign == "00111" || obstacle_sign == "00011"  || obstacle_sign == "00101" || obstacle_sign == "00110" || obstacle_sign == "01010" ) {
      Serial.println("hand left");
      go_Left(TURN_SPEED, TURN_SPEED);//Turn left
      delay(turntime);
    }

    else if (  obstacle_sign == "01111" ||  obstacle_sign == "10111" || obstacle_sign == "11111"  ) {
      Serial.println("hand back right");
      go_Left(FAST_SPEED, SPEED);
      delay(backtime);
    }
    else if ( obstacle_sign == "11011"  ||    obstacle_sign == "11101"  ||  obstacle_sign == "11110"  || obstacle_sign == "01110"  ) {
      Serial.println("hand back left");
      go_Right(SPEED, FAST_SPEED);
      delay(backtime);
    }

    else Serial.println("no handle");
    numcycles = 0; //Restart count of cycles
  } else {
    go_Advance(SPEED, SPEED);  // if nothing is wrong go forward using go() function above.
    delay(backtime);
  }

  //else  Serial.println(numcycles);

  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance < distancelimit) { // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
    Serial.println("final go back");
    go_Right(SPEED, FAST_SPEED);
    delay(backtime * 3 / 2);
    ++thereis;
  }
  if (distance > distancelimit) {
    thereis = 0;
  } //Count is restarted
  if (thereis > 25) {
    Serial.println("final stop");
    stop_Stop(); // Since something is ahead, stop moving.
    thereis = 0;
  }
}

void setup() {
  stop_Stop();//stop move

  /*init servo*/
  head.attach(servoPin);
  
  head.write(90);
  delay(2000);

  Serial.begin(9600);
}

void loop() {
  auto_avoidance();
}
