# include <EEPROM.h>
#include <math.h>

const bool READ_EEPROM = true;
const int NUM_STATES = 36;  // number of state (distance) intervals used in qtable
const int NUM_ACTIONS = 20;  // number of action (vl - vr) intervals used in qtable

int TICK = 0;

float qtable[NUM_STATES][NUM_ACTIONS] = {};



// initializes the q-table
void init_qtable() {
  for (int i = 0; i != NUM_STATES; i++) {
    for (int j = 0; j != NUM_ACTIONS; j++)
      qtable[i][j] = 0.0;
  }
}


void write_EEPROM() {
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      EEPROM.update(i * NUM_ACTIONS + j, qtable[j][i]);
    }
  }
}


void read_EEPROM() {
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      qtable[i][j] = EEPROM.read(i * NUM_ACTIONS + j);
    }
  }

  return qtable;
}


// update the qtable with new information
void update_qtable() {

  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      if (random(10) > 5)
        qtable[i][j] += random(3);
    }
  }
}


void print_qtable() {
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      Serial.print((int)qtable[i][j]);
      Serial.print('|');
    }
    Serial.println();
  }
  Serial.println();
}

void setup() {
  randomSeed(analogRead(0)); // used for mock update

  Serial.begin(9600);
  if (READ_EEPROM) {
    read_EEPROM();
  } else {
    init_qtable();
  }

  print_qtable();
}

void loop() {
  update_qtable();

  if (TICK % 100 == 0) {
    print_qtable();
    Serial.println("WRITING...");
    write_EEPROM();
  }
  Serial.println(TICK);
  TICK++;
}
