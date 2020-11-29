#include "MeAuriga.h"

MeSoundSensor mic(PORT_14);
const int SND_THLD = 50;
int sound;


MeLightSensor light_r(PORT_11);
MeLightSensor light_l(PORT_12);
const int LIGHT_THLD = 50;
int light_read_r;
int light_read_l;

bool patrol_detected = false;

bool detect_patrol() {
  sound = mic.strength();
  light_read_r = light_r.read();
  light_read_l = light_l.read();

  if (sound > SND_THLD || light_read_r > LIGHT_THLD || light_read_r > LIGHT_THLD)
    return true;
    
  return false;
}


void setup() {
  // ledPin, sensorPin
  Serial.begin(9600);
}

void loop() {

  Serial.print("LR: ");
  Serial.print(light_read_r);
  Serial.print("\tLR: ");
  Serial.print(light_read_l);
  Serial.print("\tSound: ");
  Serial.print(sound);
  Serial.print("\tPtrl det: ");
  Serial.println(detect_patrol());
    
  delay(100);
}
