# include <EEPROM.h>

const int NUM_STATES = 36;  // number of state (distance) intervals used in qtable
const int NUM_ACTIONS = 20;  // number of action (vl - vr) intervals used in qtable
const bool READ_EEPROM = false;
int num = 1;

// a representation of the world state
struct WorldState {
  float** qtable;
};

WorldState STATE;

// initializes the q-table
float** init_qtable(int num_states, int num_actions, bool read_csv) {

  float** qtable = new float*[num_states];
  if (read_csv) {
    qtable = read_EEPROM();
    return qtable;
  } else {
    for (int i = 0; i < num_states; i++) {
      qtable[i] = new float[num_actions];
      for (int j = 0; j < num_actions; j++)
        qtable[i][j] = (i * NUM_ACTIONS + j);
    }
  }
  return qtable;
}

void write_EEPROM(float** qtable) {
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      EEPROM.update(i * NUM_ACTIONS + j, qtable[j][i]);
    }
  }
}

/*
  void write_csv(char* filepath, float** qtable) {
  std::ofstream csv;
  csv.open(filepath);
  for (int i = 0; i < NUM_STATES; i++) {
    float* row = qtable[i];
    for (int j = 0; j < NUM_ACTIONS; j++) {
      csv << row[j];
      if (j < (NUM_ACTIONS - 1))
        csv << ",";
      else
        csv << "\n";
    }
  }
  }

*/

float** read_EEPROM() {
  float** qtable = new float*[NUM_STATES];

  for (int i = 0; i < NUM_STATES; i++) {
    qtable[i] = new float[NUM_ACTIONS];
    for (int j = 0; j < NUM_ACTIONS; j++) {
      qtable[i][j] = EEPROM.read(i * NUM_ACTIONS + j);
    }
  }

  return qtable;
}

/*
  float** read_from_csv(char* filepath) {
  float** qtable = new float*[NUM_STATES;
  std::ifstream csv;
  csv.open(filepath);
  while (!csv.eof()) {
    float* row = new float[NUM_ACTIONS];
    string line, val;
    getline(csv, line);
    std::stringstream ss(line);
    while (getline(ss, val, ',')) {
      row.push_back(std::stof(val));
    }
    qtable.push_back(row);
  }
  return qtable;
  }
*/


// update the qtable with new information
float** update_qtable(float** qtable) {

  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      qtable[i][j] += 1;
    }
  }

  return qtable;
}


void print_qtable(float** qtable) {
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
  Serial.begin(9600);

  STATE.qtable = init_qtable(NUM_STATES, NUM_ACTIONS, READ_EEPROM);
  
  Serial.println("set up");
}

void loop() {
  STATE.qtable = update_qtable(STATE.qtable);
  Serial.println(num);
  if (num % 500 == 0) {
    print_qtable(STATE.qtable);
    write_EEPROM(STATE.qtable);
  }
  num++;
}
