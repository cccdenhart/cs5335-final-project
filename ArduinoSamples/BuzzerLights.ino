#include "MeAuriga.h"

MeBuzzer siren;
#define BUZZER_PORT 45

MeRGBLed leds(0);
#define LEDNUM 12
#define LEDPORT 44
#define BLUE 0, 0, 255
#define RED 255, 0, 0
short int col = 0;

MeSoundSensor mic(PORT_14);
int16_t sound;


void setup() {
  Serial.begin(9600);
  
  siren.setpin(BUZZER_PORT);
  siren.noTone();
  
  leds.setpin(LEDPORT);
  leds.setNumber(LEDNUM);
  leds.setColor(0, 0, 0);
}

void loop() {
  for (int i = 0; i < 8; i++) {
    chase();
    sound = mic.strength();
    Serial.println(sound);
  }
  
  leds.setColor(0, 0, 0);
  leds.show();
  siren.noTone();
  delay(2000);
  sound = mic.strength();
  Serial.println(sound);
}

void chase() {
  if (col == 0) {
    leds.setColor(RED);
    leds.show();
    siren.tone(523, 500);   //C5
    col++;
  } else {
    leds.setColor(BLUE);
    leds.show();
    siren.tone(370, 500);   //F#4
    col--;
  }
}
