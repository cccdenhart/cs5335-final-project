/*
   TODO:
   - add key condition (short on both sonars, line follower on) + track keys
   - incorporate sound
   - presentation + project text file
   - gazebo video
   - keyboard input training?
*/

#include <math.h>
#include "MeAuriga.h"

MeRGBLed led;

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
const float RG_MAX_DIST = 100.0;  // ranger max distance considered for state
const int NUM_STATES = 64;  // number of state (distance) intervals used in qtable
const int NUM_ACTIONS = 20;  // number of action (vl - vr) intervals used in qtable
bool TRAINING = false;
float EPS = 0.7;
int TICK = 0;
float dist_f = 0;
float dist_r = 0;
int old_int_state = 0;
int next_int_action = 0;
int cur_int_state = 0;
float reward = 0;
float cur_vr = 0;
float cur_vl = 0;
float** qtable;
float old_dist_r = 0;
float old_dist_f = 0;
const int SND_THLD = 1500; // 550
const int LIGHT_THLD = 1500; // 100

int sound;
int light_read_r;
int light_read_l;

bool patrol_detected = false;

// Line follower
MeLineFollower lineSensor(PORT_10);

// Buzzer
#define BUZZER_PORT 45

const float MAX_HALL = 15.0;
bool INSIDE = false;
bool DONE = false;
int key_count = 0;

MeEncoderOnBoard MOTOR_R(SLOT_1);
MeEncoderOnBoard MOTOR_L(SLOT_2);

MeLineFollower lineSensorFront(PORT_10);

MeUltrasonicSensor ULTRA_F(PORT_9);
MeUltrasonicSensor ULTRA_R(PORT_7);
MeSoundSensor mic(PORT_14);

MeLightSensor light_r(PORT_11);
MeLightSensor light_l(PORT_12);

MeBuzzer siren;
#define BUZZER_PORT 45

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

void win()
{
  // optionally do stop motors dim the LED's etc.
  // Serial.print("stopped"); // or other warning
  Serial.println("YOU WIN!!");
  while (1) {
    led.setColor(0, 50, 0, 0);
    led.show();
    delay(50);
    led.setColor(0, 0, 50, 0);
    led.show();
    delay(50);
    led.setColor(0, 0, 0, 50);
    led.show();
  } ;
}
bool detect_patrol() {
  sound = mic.strength();
  light_read_r = light_r.read();
  light_read_l = light_l.read();

  if (sound > SND_THLD || light_read_r > LIGHT_THLD || light_read_r > LIGHT_THLD)
    return true;

  return false;
}


// limit the given value between the given min/max
float clamp(float xmin, float xx, float xmax) {
  if (xx < xmin) return xmin;
  if (xx > xmax) return xmax;
  return xx;
}

// converts a raw state representation to an integer representation
// (usable by the Q-Table)
int discretize_state() {
  int root_states = floor(sqrt(NUM_STATES));

  float dist_f_local = clamp(0.0, dist_f, RG_MAX_DIST);
  int norm_f = floor(dist_f_local / RG_MAX_DIST) * (root_states - 1);

  float dist_r_local = clamp(0.0, dist_r, RG_MAX_DIST);
  int norm_r = floor(dist_r_local / RG_MAX_DIST) * (root_states - 1);

  float result = norm_f + root_states * norm_r;

  return result;
}

// convert an integer action representation to a QAction
// (usable for performing the action)
void realize_action(int int_action) {
  float diff = ((int_action * 1.0 / NUM_ACTIONS) * (RG_MAX_VEL - RG_MIN_VEL)) + RG_MIN_VEL;
  float vl = RG_BASE_VEL - diff;
  float vr = (RG_BASE_VEL + diff) * -1.0;
  cur_vl = vl;
  cur_vr = vr;
}

// initializes the q-table
void init_qtable() {
  qtable = new float*[NUM_STATES];
  for (int i = 0; i != NUM_STATES; i++) {
    qtable[i] = new float[NUM_ACTIONS];
    for (int j = 0; j != NUM_ACTIONS; j++)
      qtable[i][j] = 0.0;
  }
}

// choose a random number between 0 and 1
float rand_uniform() {
  return (float(rand()) / float(RAND_MAX)) * 1.0;
}

// choose an action
// is chosen either optimally by the qtable or randomly
// at a rate set by `eps`
int choose_action() {

  int int_action;
  float rand_val = rand_uniform();

  if (rand_val < EPS) {
    float* possible_actions = qtable[old_int_state];
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
void update_qtable() {
  float old_value = qtable[old_int_state][next_int_action];
  float max_q = -10000.0;
  for (int i = 0; i < NUM_ACTIONS; i++) {
    if (qtable[cur_int_state][i] > max_q) {
      max_q = qtable[cur_int_state][i];
    }
  }
  float new_value = (1 - ALPHA) * old_value + ALPHA * (reward + GAMMA * max_q);

  qtable[old_int_state][next_int_action] = new_value;
}

// determine a reward given the current state
// NOTE: all distances in CM (ranger values)
// rewards are arbitrary floats
// all values determined experimentally
float get_reward() {
  /* penalize if:
    1. front wall very close
    2. right wall very close
    3. right wall very far
  */

  if (dist_r < 20.0 && dist_r > 5.0 && dist_f > 15.0) {
    if ((old_dist_f > dist_f) && (old_dist_r > dist_r)) {
      return 5.0;
    }
    return 1.0;
  }

  if (dist_f < 15.0) {
    if (old_dist_f > dist_f)
      return 1.0;
    else
      return -5.0;
  }

  if (dist_r < 10.0 || dist_r > 20.0)
    return -5.0;

  return 0;
}

void checkKey() {
  if (dist_f < MAX_HALL && dist_r < MAX_HALL) {
    INSIDE = true;
  } else if (dist_f > 150 || dist_r > 150) {
    DONE = false;
  } else {
    INSIDE = false;
  }

  if (INSIDE == true && DONE == false) {
    if (lineSensor.readSensors() <= 2) {
      key_count += 1;
      DONE = true;
      led.setColor(key_count, 50, 50, 50);
      led.show();
      // siren.tone(784, 50);
      // siren.tone(1046, 100);
    }
  }
}

// setup arduino board
void setup() {
  Serial.begin(9600);
  print_qtable();
  led.setpin(44);
  Serial.println("initializing qtable .....");
  //init_qtable(NUM_STATES, NUM_ACTIONS, CSV_FILEPATH, false);
  /*leds.setpin(LEDPORT);
    leds.setNumber(LEDNUM);
    leds.setColor(OFF);
    leds.show();*/
  siren.setpin(BUZZER_PORT);
  siren.noTone();
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

  // retrieve current distances
  delay(50);
  dist_f = ULTRA_F.distanceCm();
  dist_r = ULTRA_R.distanceCm();

  old_dist_r = dist_r;
  old_dist_f = dist_f;

  led.setColor(12, 50, 0, 0);
  led.show();
  // increment tick
  TICK += 1;

  // decrease 'randomness' in q-learning actions after 5000 tick
  if (TICK > 500) {
    EPS = 0.7;
    led.setColor(11, 0, 50, 0);
    led.show();
  }
  // remove 'randomness' in q-learning actions after 15000 tick
  if (TICK > 1000) {
    EPS = 0.9;
    led.setColor(10, 0, 0, 50);
    led.show();
  }

  // check for a key
  checkKey();
  
  // find the best action to take next
  old_int_state = discretize_state();
  next_int_action = choose_action();
  realize_action(next_int_action);

  // safety mechanism for robot getting stuck on wall
  if (dist_f < 15) {
    cur_vl = -200.0;
    cur_vr = -100.0;
    reward = 0;
    MOTOR_L.setMotorPwm(cur_vl);
    MOTOR_R.setMotorPwm(cur_vr);
    //delay(200);
  } else {

    // perform that action
    MOTOR_L.setMotorPwm(cur_vl);
    MOTOR_R.setMotorPwm(cur_vr);

    // measure reward from previous action
    delay(150);
    dist_f = ULTRA_F.distanceCm();
    dist_r = ULTRA_R.distanceCm();
    // struct QState cur_state = { dist_f, dist_r };
    cur_int_state = discretize_state();
    reward = get_reward();
  }

  // update q table
  update_qtable();

  if (key_count == 8) {
    win();
  }
  // log activity
  Serial.print("tick: ");
  Serial.print(TICK);

  Serial.print("\tstate: ");
  Serial.print(cur_int_state);
  Serial.print(", (");
  Serial.print(dist_f);
  Serial.print(", ");
  Serial.print(dist_r);
  Serial.print(")");

  Serial.print("\taction: ");
  Serial.print(next_int_action);
  Serial.print(", (");
  Serial.print(cur_vl);
  Serial.print(", ");
  Serial.print(cur_vr);
  Serial.print(")");

  Serial.print("\treward: ");
  Serial.println(reward);

  Serial.print(dist_f);
  Serial.print(", ");
  Serial.print(dist_r);
  Serial.print("\tInside: ");
  Serial.print(INSIDE);
  Serial.print("\tDone: ");
  Serial.print(DONE);
  Serial.print("\tKey count: ");
  Serial.println(key_count);
  Serial.println(lineSensor.readSensors());
}
