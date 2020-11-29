#include "MeAuriga.h"

MeEncoderOnBoard MOTOR_R(SLOT_1);
MeEncoderOnBoard MOTOR_L(SLOT_2);

MeSoundSensor mic(PORT_14);

MeLightSensor light_r(PORT_11);
MeLightSensor light_l(PORT_12);

MeBuzzer siren;
#define BUZZER_PORT 45

const int SND_THLD = 550;
const int LIGHT_THLD = 100;

int sound;
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

  siren.setpin(BUZZER_PORT);
  siren.noTone();
  
  MOTOR_R.setMotorPwm(-160);
  MOTOR_L.setMotorPwm(160);
}

void loop() {
  patrol_detected = detect_patrol();

  if (patrol_detected) {
    MOTOR_R.setMotorPwm(0);
    MOTOR_L.setMotorPwm(0);
    siren.tone(523, 100);
    siren.tone(370, 100);
    delay(3000);
  }

  
  Serial.print("LR: ");
  Serial.print(light_read_r);
  Serial.print("\tLR: ");
  Serial.print(light_read_l);
  Serial.print("\tSound: ");
  Serial.print(sound);
  Serial.print("\tPtrl det: ");
  Serial.println(patrol_detected);  
  delay(50);
}
