// Two devices can be connected
//Servo myservo2;  // create servo object to control another servo
//int16_t servo2pin =  port.pin2();//attaches the servo on PORT_3 SLOT2 to the servo object
//myservo2.attach(servo2pin);  // attaches the servo on servopin2
//myservo2.write(0);
//myservo2.write(180);

#include "MeAuriga.h"

MeUltrasonicSensor ultra(PORT_8);
double distanceL;
double distanceR;
double distanceF;

MePort port(PORT_9);
Servo servo;  // create servo object to control a servo
int16_t servoPin =  port.pin1();//attaches the servo on PORT_3 SLOT1 to the servo object


void setup()
{
  Serial.begin(9600);
  servo.attach(servoPin);  // attaches the servo on servopin1
}

void loop()
{
  servo.write(90);
  delay(500);
  distanceF = ultra.distanceCm();
  
  Serial.print("Front: ");
  Serial.println(distanceF);

  if (distanceF < 20) {
    Serial.println("Checking surroungings");
    
    servo.write(0);
    delay(500);
    distanceR = ultra.distanceCm();

    servo.write(180);
    delay(500);
    distanceL = ultra.distanceCm();

    Serial.print("Left: ");
    Serial.print(distanceL);
    Serial.print("   ");
    Serial.print("Right: ");
    Serial.println(distanceR);
  }

  delay(50);
}
