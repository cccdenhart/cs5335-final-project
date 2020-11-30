# include <EEPROM.h>

const bool READ_EEPROM = true;
const int NUM_STATES = 36;  // number of state (distance) intervals used in qtable
const int NUM_ACTIONS = 10;  // number of action (vl - vr) intervals used in qtable

int TICK = 0;

float qtable[NUM_STATES][NUM_ACTIONS] = {};



// initializes the q-table
void init_qtable() {
  for (int i = 0; i != NUM_STATES; i++) {
    for (int j = 0; j != NUM_ACTIONS; j++)
      qtable[i][j] = 0;
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
}


// update the qtable with new information
void update_qtable() {

  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      if (i % 2 == 0) {
        qtable[i][j] += 0.5;
      } else {
        qtable[i][j] -= 0.5;
      }
    }
  }
}


void print_qtable() {
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      Serial.print(qtable[i][j]);
      Serial.print('|');
    }
    Serial.println();
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);

  if (READ_EEPROM) {
    read_EEPROM();
  } else {
    init_qtable();
  }

}

void loop() {
  if (TICK % 5 == 0) {
    print_qtable();
    Serial.println("WRITING...");
    write_EEPROM();
  }
  update_qtable();
  Serial.println(TICK);
  TICK++;
  delay(2000);
}
