// Led Ring
MeRGBLed leds(0);
#define LEDNUM 12
#define LEDPORT 55
#define OFF 0, 0, 0
#define RED 255, 0, 0
#define BLUE 0, 0, 255
#define GREEN 0, 255, 0

// Line follower
MeLineFollower lineSensor(PORT_10);

// Buzzer
#define BUZZER_PORT 45

const float MAX_HALL = 114;
bool INSIDE = false;
bool DONE = false;
int keyCount = 0;


void setup() {
  leds.setpin(LEDPORT);
  leds.setNumber(LEDNUM);
  leds.setColor(OFF);
  leds.show();

  siren.setpin(BUZZER_PORT);
  siren.noTone();
}


void checkKey() {
  if (dist_f < MAX_HALL) {
    INSIDE = true;
  } else {
    INSIDE = false;
  }
  
  if (INSIDE == true && DONE == false) {
    if (lineSensor.readSensors() == 0) {
      DONE = true;
      keyCount += 1;
      //leds.setColorAt(keyCount);
      //siren.tone(784, 50);
      //siren.tone(1046, 100);
    }
  } else {
    DONE = false;
  }
}

void loop() {
  checkKey();
}
