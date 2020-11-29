// #include <iostream>
//#include <string>
//#include <filesystem>
#include <math.h>
#include <string.h>
//#include <cstdlib>
//#include <fstream>
#include "MeAuriga.h"

//#include "robot.hh"

#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77

//using std::string;
// using std::vector;
//using std::cout;
//using std::endl;

const bool READ_CSV = false;
//const char[] CSV_FILEPATH = "qtable.csv";
const float ALPHA = 0.2;
const float GAMMA = 0.9;
const float BASE_VEL = 2.0;
const float MIN_VEL = -3.0;
const float MAX_VEL = 3.0;
const int NUM_STATES = 20;
const int NUM_ACTIONS = 20;
bool TRAINING = false;
float EPS = 0.3;
int TICK = 0;


// Motors
MeEncoderOnBoard motorR(SLOT_1);
MeEncoderOnBoard motorL(SLOT_2);

// Ultrasonic Sensors
MeUltrasonicSensor ultra(PORT_9);
MeUltrasonicSensor ultra2(PORT_7);

// Line Sensor
MeLineFollower lineSensorFront(PORT_10);


// a raw representation of a robot state
struct QState {
  float dist;
  float dist2;
};

// a raw representation of a robot action
struct QAction {
  float vl;
  float vr;
};

// a representation of the world state
struct WorldState {
  float** qtable;
};

WorldState STATE;

float
clamp(float xmin, float xx, float xmax)
{
    if (xx < xmin) return xmin;
    if (xx > xmax) return xmax;
    return xx;
}

// converts a raw state representation to an integer representation
// (usable by the Q-Table)
int discretize_state(QState state) {
  float max_dist = 3.0;
  float dist = clamp(0.0, state.dist, max_dist);
  int root_states = floor(sqrt(NUM_STATES));
  int norm = floor(dist / max_dist * root_states);

  float dist2 = clamp(0.0, state.dist, max_dist);
  int norm2 = floor(dist / max_dist * root);

  return norm;
}

// converts a raw action representation to an integer representation
// (usable by the Q-Table)
int discretize_action(QAction action) {
  float diff = clamp(BASE_VEL + MIN_VEL, action.vl - action.vr, BASE_VEL + MAX_VEL);
  int norm = floor((diff - MIN_VEL) / (MAX_VEL - MIN_VEL) * NUM_ACTIONS);
  return norm;
}

// convert an integer action representation to a QAction
// (usable for performing the action)
QAction realize_action(int int_action) {
  float diff = ((int_action * 1.0 / NUM_ACTIONS) * (MAX_VEL - MIN_VEL)) + MIN_VEL;
  float vl = BASE_VEL - diff;
  float vr = BASE_VEL + diff;
  return { vl, vr };
}

/*
void write_csv(char[] filepath, float[][] qtable) {
  std::ofstream csv;
  csv.open(filepath);
  for (int i = 0; i < NUM_STATES; i++) {
    float[NUM_ACTIONS] row = qtable[i];
    for (int j = 0; j < NUM_ACTIONS; j++) {
      csv << row[j];
      if (j < (row.size() - 1))
        csv << ",";
      else
        csv << "\n";
    }
  }
}
*/

/*
float[][] read_from_csv(string filepath) {
  float[NUM_STATES][NUM_ACTIONS] qtable;
  std::ifstream csv;
  csv.open(filepath);
  while (!csv.eof()) {
    vector<float> row;
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

// initializes the q-table
float** init_qtable(int num_states, int num_actions/*, char[] filepath, */, bool read_csv = false) {
  float** qtable = new float*[num_states];
  if (read_csv)
    //qtable = read_from_csv(filepath);
    return qtable;
  else {
    for (int i = 0; i != num_states; i++) {
        qtable[i] = new float[NUM_ACTIONS];
        for (int j = 0; j != num_actions; j++)
            qtable[i][j] = 0.0;
    }
  }
  return qtable;
}

// choose a random number between 0 and 1
float rand_uniform() {
  return (float(rand()) / float(RAND_MAX)) * 1.0;
}

// choose an action
// is chosen either optimally by the qtable or randomly
// at a rate set by `eps`
int choose_action(
    float** qtable,
    QState state,
    float eps = EPS
) {

  int int_state = discretize_state(state);
  int int_action;
  float rand_val = rand_uniform();

  if (rand_val < eps) {
    float* possible_actions = qtable[int_state];
    int int_action = 0;
    float max_action = 0.0;
    for (int i = 0; i < NUM_ACTIONS; i++) {
      if (possible_actions[i] > max_action) {
        int_action = i;
        max_action = possible_actions[i];
      }
    }
  }
  else {
    int_action = floor(rand_uniform() * NUM_ACTIONS);
  }
  
  return int_action;
}

// update the qtable with new information
float** update_qtable(
    float** qtable,
    int action,
    float reward,
    int current_state,
    int old_state,
    float learning_rate = ALPHA,
    float reward_decay = GAMMA
) {
  float old_value = qtable[old_state][action];
  float max_q = -1000.0;
  for (int i = 0; i < NUM_ACTIONS; i++) {
    if (qtable[current_state][i] > max_q) {
      max_q = qtable[current_state][i];
    }
  }
  float new_value = (1 - learning_rate) * old_value + learning_rate * (reward + reward_decay * max_q);

  qtable[old_state][action] = new_value;

  return qtable;
}

// determine a reward given the current state
float get_reward(QState state) {
  if (state.dist < 0.3)
    return -5.0;
  else if (state.dist < 0.6)
    return -1.0;
  else if (state.dist < 1.0)
    return 1.0;
  else if (state.dist < 1.5)
    return 5.0;
  else if (state.dist < 2.5)
    return 1.0;
  else
    return -5.0;
}

/*
void keyboard_input(Robot* robot) {
  switch((c=getch())) {
    case KEY_UP:
      cout << endl << "Up" << endl;//key up
      robot->set_vel(2.0, 2.0);
    case KEY_DOWN:
      cout << endl << "Down" << endl;   // key down
      robot->set_vel(-2.0, -2.0);
    case KEY_LEFT:
      cout << endl << "Left" << endl;  // key left
      robot->set_vel(-2.0, 2.0);
    case KEY_RIGHT:
      cout << endl << "Right" << endl;  // key right
      robot->set_vel(2.0, -2.0);
    case "d":
      cout << "done training" << endl;
      TRAINING = FAlSE;
    default:
      cout << endl << "null" << endl;  // not arrow
      break;
  } 
}
*/

void setup() {
  Serial.begin(9600);
  Serial.println("initializing qtable");
  STATE.qtable = init_qtable(NUM_STATES, NUM_ACTIONS/*, CSV_FILEPATH, READ_CSV*/);
}

QAction normalize_speed(QAction action) {
  float rg_min = -40;
  float rg_max = 200;
  float gz_min = -1.0;
  float gz_max = 5.0;
  float norm_vl = (action.vl - gz_min) / (gz_max - gz_min);
  float new_vl = norm_vl * (rg_max - rg_min) + rg_min;
  float norm_vr = (action.vr - gz_min) / (gz_max - gz_min);
  float new_vr = -1 * (norm_vr * (rg_max - rg_min) + rg_min);
  // return { map(action.vl, -1.0, 5.0, -200, 200), map(action.vr, -1.0, 5.0, -200, 200) };
  return { new_vl, new_vr };
}

float normalize_dist(float rg_dist) {
  float norm_dist = (rg_dist) / 400.0 * 2.75;
  return norm_dist;
}

void loop() {
  float dist = normalize_dist(ultra.distanceCm());
  float dist2 = normalize_dist(ultra2.distanceCm());
    // increment tick
  TICK += 1;

  if (TRAINING)
    // keyboard_input(robot);
    return;
  else {

    // decrease 'randomness' in q-learning after 5000 tick
    if (TICK > 5000)
      EPS = 0.9;

    // cache qtable
    /*
    if (TICK % 1000 == 0)
      write_csv(CSV_FILEPATH, STATE.qtable);
      */

      
    // find the best action to take next
    QState old_state = { dist };
    int old_int_state = discretize_state(old_state);
    int next_int_action = choose_action(STATE.qtable, old_state);
    QAction next_action = realize_action(next_int_action);

    // safety mechanism for robot getting stuck on wall
    if (dist < 0.1)
      next_action = { -2.0, -3.0 };

    // default behavior when no wall in sight
    if (dist > 1.8)
      next_action = { 0.5, 4.0 };

    // perform that action
    
    QAction next_rg_action = normalize_speed(next_action);
    motorL.setMotorPwm(next_rg_action.vl);
    motorR.setMotorPwm(next_rg_action.vr);

    // measure reward from previous action
    QState cur_state = { dist };
    int cur_int_state = discretize_state(cur_state);
    float reward = get_reward(cur_state);

    // update q table
    STATE.qtable = update_qtable(STATE.qtable, next_int_action, reward, cur_int_state, old_int_state);


    Serial.println("tick: " + TICK);
    
    Serial.print("State: ");
    Serial.print(cur_int_state);
    Serial.print(", ");
    Serial.println(cur_state.dist);
    
    Serial.print("action: ");
    Serial.print(next_int_action);
    Serial.print(", (");
    Serial.print(next_action.vl);
    Serial.print(", ");
    Serial.print(next_action.vr);
    Serial.print(" ), (");
    Serial.print(next_rg_action.vl);
    Serial.print(", ");
    Serial.print(next_rg_action.vr);
    Serial.println(")");

    Serial.print("reward: ");
    Serial.println(reward);
    
    Serial.println("=====\n");
    delay(600);
    /*
    cout << "tick: " << TICK << endl;
    cout << "state: " << cur_int_state << ", " << cur_state.dist << endl;
    cout << "action: " 
         << next_int_action 
         << ", " 
         << "(" << next_action.vl << ", " << next_action.vr << ")" 
         << endl;
    cout << "reward: " << reward << endl;
    cout << "=====\n" << endl;
    */
  }
}
