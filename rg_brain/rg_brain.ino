/*
 * TODO:
 * - add key condition (short on both sonars, line follower on) + track keys
 * - incorporate sound
 * - presentation + project text file
 * - gazebo video
 * - keyboard input training?
 */

#include <math.h>
#include <string.h>
#include "MeAuriga.h"

/*
#include "robot.hh"
using std::string;
using std::vector;
using std::cout;
using std::endl;
*/

const bool READ_CSV = false;
const char* CSV_FILEPATH = "qtable.csv";
const float ALPHA = 0.2;
const float GAMMA = 0.9;
const float GZ_BASE_VEL = 2.0;  // gazebo base velocity
const float GZ_MIN_VEL = -3.0;  // gazebo most negative amount CHANGED from base vel
const float GZ_MAX_VEL = 3.0;  // gazebo most postive amount CHANGED from base vel
const float RG_BASE_VEL = 100.0;  // ranger base velocity
const float RG_MIN_VEL = -150.0;  // ranger most negative amount CHANGED from base vel
const float RG_MAX_VEL = 150.0;  // ranger most positive amount CHANGED from base vel
const float RG_MAX_DIST = 200.0;  // ranger max distance considered for state
const int NUM_STATES = 64;  // number of state (distance) intervals used in qtable
const int NUM_ACTIONS = 20;  // number of action (vl - vr) intervals used in qtable 
bool TRAINING = false;
float EPS = 0.3;
int TICK = 0;

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

const float MAX_HALL = 15.0;
bool INSIDE = false;
bool DONE = false;
int keyCount = 0;

MeEncoderOnBoard MOTOR_R(SLOT_1);
MeEncoderOnBoard MOTOR_L(SLOT_2);

MeLineFollower lineSensorFront(PORT_10);

MeUltrasonicSensor ULTRA_F(PORT_9);
MeUltrasonicSensor ULTRA_R(PORT_7);

// a raw representation of a robot state
struct QState {
  float dist_f;
  float dist_r;
};

// a raw representation of a robot action
struct QAction {
  float vl;
  float vr;
};

// a representation of the world state
struct WorldState {
  float** qtable;
  int remaining_keys = 5;
};

WorldState STATE;

// limit the given value between the given min/max
float clamp(float xmin, float xx, float xmax) {
  if (xx < xmin) return xmin;
  if (xx > xmax) return xmax;
  return xx;
}

// converts a raw state representation to an integer representation
// (usable by the Q-Table)
int discretize_state(QState state) {
  int root_states = floor(sqrt(NUM_STATES));
  
  float dist_f = clamp(0.0, state.dist_f, RG_MAX_DIST);
  int norm_f = ceil((dist_f / RG_MAX_DIST) * root_states);

  float dist_r = clamp(0.0, state.dist_r, RG_MAX_DIST);
  int norm_r = ceil((dist_r / RG_MAX_DIST) * root_states);

  return norm_f * norm_r;
}

// converts a raw action representation to an integer representation
// (usable by the Q-Table)
int discretize_action(QAction action) {
  float diff = clamp(RG_BASE_VEL + RG_MIN_VEL, action.vl - action.vr, RG_BASE_VEL + RG_MAX_VEL);
  int norm = floor((diff - RG_MIN_VEL) / (RG_MAX_VEL - RG_MIN_VEL) * NUM_ACTIONS);
  return norm;
}

// convert an integer action representation to a QAction
// (usable for performing the action)
QAction realize_action(int int_action) {
  float diff = ((int_action * 1.0 / NUM_ACTIONS) * (RG_MAX_VEL - RG_MIN_VEL)) + RG_MIN_VEL;
  float vl = RG_BASE_VEL - diff;
  float vr = RG_BASE_VEL + diff;
  return { vl, vr };
}

// convert gazebo action values to ranger action values
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

// convert gazebo distance values to ranger distance values
float normalize_dist(float rg_dist) {
  float norm_dist = (rg_dist) / 400.0 * 2.75;
  return norm_dist;
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

// initializes the q-table
float** init_qtable(int num_states, int num_actions, const char* filepath, bool read_csv) {
  float** qtable = new float*[num_states];
  if (read_csv)
    // qtable = read_from_csv(filepath);
    return qtable;
  else {
    for (int i = 0; i != num_states; i++) {
        qtable[i] = new float[num_actions];
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
  float max_q = -10000.0;
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
// NOTE: all distances in CM (ranger values)
// rewards are arbitrary floats
// all values determined experimentally
float get_reward(QState state, QAction action) {
  /* penalize if:
   * 1. front wall very close
   * 2. right wall very close
   * 3. right wall very far
   */
  if (state.dist_f < 5.0 || state.dist_r < 5.0 || state.dist_r > 20.0)
    return -5.0;
 
  if (state.dist_r < 20.0) {
    // best reward if wall close and positive velocity
    if (action.vl > 0 && action.vr > 0)
      return 5.0;
    // otherwise, return mild reward
    return 1.0;
  }

  return 0;
}

/*
// allow controlling robot via keyboard input
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

void checkKey(float dist_f, float dist_r) {
  if (dist_f < MAX_HALL && dist_r < MAX_HALL) {
    INSIDE = true;
  } else {
    INSIDE = false;
  }
  
  if (INSIDE == true && DONE == false) {
    if (lineSensor.readSensors() == 0) {
      DONE = true;
      STATE.remaining_keys -= 1;
      // leds.setColorAt(keyCount);
      // siren.tone(784, 50);
      // siren.tone(1046, 100);
    } else {
    DONE = false;
    }
  } 
}

// setup arduino board
void setup() {
  Serial.begin(9600);
  Serial.println("initializing qtable .....");
  STATE.qtable = init_qtable(NUM_STATES, NUM_ACTIONS, CSV_FILEPATH, false);
  leds.setpin(LEDPORT);
  leds.setNumber(LEDNUM);
  leds.setColor(OFF);
  leds.show();
}

void loop() {
  // retrieve current distances
  float dist_f = ULTRA_F.distanceCm();
  float dist_r = ULTRA_R.distanceCm();
  
  // increment tick
  TICK += 1;

  // if training, only process keyboard input
  // else, use qtable
  // NOTE: keyboard functionality not yet enabled
  if (TRAINING)
    // keyboard_input(robot);
    return;
  else {

    // decrease 'randomness' in q-learning actions after 5000 tick
    if (TICK > 100)
      EPS = 0.9;
    // remove 'randomness' in q-learning actions after 15000 tick
    if (TICK > 500)
      EPS = 1.0;

    // check for a key
    checkKey(dist_f, dist_r);

    // cache qtable
    /*
    if (TICK % 1000 == 0)
      write_csv(CSV_FILEPATH, STATE.qtable);
    */
      
    // find the best action to take next
    QState old_state = { dist_f, dist_r };
    int old_int_state = discretize_state(old_state);
    int next_int_action = choose_action(STATE.qtable, old_state);
    QAction next_action = realize_action(next_int_action);

    // safety mechanism for robot getting stuck on wall
    if (dist_f < 5.0)
      next_action = { -100.0, -200.0 };

    // perform that action
    MOTOR_L.setMotorPwm(next_action.vl);
    MOTOR_R.setMotorPwm(next_action.vr);

    // measure reward from previous action
    delay(60);
    float dist_f = ULTRA_F.distanceCm();
    float dist_r = ULTRA_R.distanceCm();
    QState cur_state = { dist_f, dist_r };
    int cur_int_state = discretize_state(cur_state);
    float reward = get_reward(cur_state, next_action);

    // update q table
    STATE.qtable = update_qtable(STATE.qtable, next_int_action, reward, cur_int_state, old_int_state);

    // log activity
    Serial.print("tick: ");
    Serial.print(TICK);
    
    Serial.print("\tstate: ");
    Serial.print(cur_int_state);
    Serial.print(", (");
    Serial.print(cur_state.dist_f);
    Serial.print(", ");
    Serial.print(cur_state.dist_r);
    Serial.print(")");
    
    Serial.print("\taction: ");
    Serial.print(next_int_action);
    Serial.print(", (");
    Serial.print(next_action.vl);
    Serial.print(", ");
    Serial.print(next_action.vr);
    Serial.print(")");

    Serial.print("\treward: ");
    Serial.println(reward);
  }
}
