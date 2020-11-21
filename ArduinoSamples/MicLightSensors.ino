#include "MeAuriga.h"

MeSoundSensor mic(PORT_14);
int16_t sound;

MeLightSensor lightR2(PORT_11);
MeLightSensor lightL1(PORT_12);
int16_t lightRdR;
int16_t lightRdL;

void setup() {
  // ledPin, sensorPin
  Serial.begin(9600);
}

void loop() {
  sound = mic.strength();
  Serial.println(sound);

  lightRdR = lightR2.read();
  lightRdL = lightL1.read();
  
  Serial.print(lightRdR);
  Serial.print(" ");
  Serial.println(lightRdL);
  
  delay(100);
}
