/*
   CS5335 - Robotic Science
   Final Project
   Fall, 2020

   Charles Denhart
   Pablo Gomez Forero
   Idakelly King
   Amit Vijaykumar Mulay
*/

#include <EEPROM.h>
#include <math.h>
#include "MeAuriga.h"

/*******************************************************/
//////// ***** HARDWARE DECLARATIONS START ***** ////////
/*******************************************************/

// Motors
MeEncoderOnBoard MOTOR_R(SLOT_1);
MeEncoderOnBoard MOTOR_L(SLOT_2);

// Line follower
MeLineFollower lineSensor(PORT_10);

// Ultrasonic sensors
MeUltrasonicSensor ULTRA_F(PORT_9);
MeUltrasonicSensor ULTRA_R(PORT_7);

// Sound sensor
MeSoundSensor mic(PORT_14);

// Light sensor
MeLightSensor light_r(PORT_11);
MeLightSensor light_l(PORT_12);

// Buzzer
MeBuzzer siren;
#define BUZZER_PORT 45

// Led Ring
MeRGBLed led;
#define LED_NUM  12
#define LED_PORT 44
#define RED     255, 0, 0
#define GREEN   0, 255, 0
#define BLUE    0, 0, 255
#define WHITE   255, 255, 255
#define OFF     0, 0, 0

//////// ***** Hardware declarations END ***** ////////


/**************************************************/
//////// ***** GLOBAL VARIABLES START ***** ////////
/**************************************************/

// Q-Learning constants
const float ALPHA = 0.2;
const float GAMMA = 0.9;
const float GZ_BASE_VEL = 2.0;    // gazebo base velocity
const float GZ_MIN_VEL = -3.0;    // gazebo most negative amount CHANGED from base vel
const float GZ_MAX_VEL = 3.0;     // gazebo most postive amount CHANGED from base vel
const float RG_BASE_VEL = 100.0;  // ranger base velocity
const float RG_MIN_VEL = -150.0;  // ranger most negative amount CHANGED from base vel
const float RG_MAX_VEL = 150.0;   // ranger most positive amount CHANGED from base vel
const float RG_MAX_DIST = 36;  // ranger max distance considered for state // 100
const int NUM_STATES = 36;        // number of state (distance) intervals used in qtable // 36
const int NUM_ACTIONS = 7;       // number of action (vl - vr) intervals used in qtable //10
const int ACTION_DELAY = 200;    // 150

// Q-Learning variables
bool TRAINING = false;
float EPS = 0.7;
int TICK = 0;
int old_int_state = 0;
int next_int_action = 0;
int cur_int_state = 0;
float reward = 0;
float cur_vr = 0;
float cur_vl = 0;
float dist_f = 0;
float dist_r = 0;
float old_dist_r = 0;
float old_dist_f = 0;
float qtable[NUM_STATES][NUM_ACTIONS] = {};

// Key finding
const float MAX_HALL = 15.0;
bool INSIDE = false;
bool DONE = false;
int key_count = 0;

// Detect patrol
const int SND_THLD = 1500; // 550
const int LIGHT_THLD = 1500; // 100
int sound;
int light_read_r;
int light_read_l;
bool patrol_detected = false;

// Load from non-volatile memory
const bool READ_EEPROM = false;

//////// ***** Global variables END ***** ////////


/******************************************************/
//////// ***** Q-LEARNING FUNCTIONS START ***** ////////
/******************************************************/

// initialize q-table to 0s
void init_qtable() {
  for (int i = 0; i != NUM_STATES; i++) {
    for (int j = 0; j != NUM_ACTIONS; j++)
      qtable[i][j] = 0.0;
  }
}


// update qtable with new information
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


// converts a raw state representation to an integer representation
int discretize_state() {
  int root_states = floor(sqrt(NUM_STATES));

  float dist_f_local = clamp(0.0, dist_f, RG_MAX_DIST);
  int norm_f = floor((dist_f_local / RG_MAX_DIST) * (root_states - 1));

  float dist_r_local = clamp(0.0, dist_r, RG_MAX_DIST);
  int norm_r = floor((dist_r_local / RG_MAX_DIST) * (root_states - 1));

  float result = norm_f + root_states * norm_r;

  return result;
}


// choose an action
// either optimally by the qtable or randomly at as per `eps`
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

/*
  // convert an integer action representation to a QAction
  void realize_action(int int_action) {
  float diff = ((int_action * 1.0 / NUM_ACTIONS) * (RG_MAX_VEL - RG_MIN_VEL)) + RG_MIN_VEL;
  float vl = RG_BASE_VEL - diff;
  float vr = (RG_BASE_VEL + diff) * -1.0;
  cur_vl = vl;
  cur_vr = vr;
  }
*/


void realize_action(int int_action)
{
  int pwm = 180;
  switch (int_action)
  {
    case 0: //Correct left slight
      cur_vl =  pwm - pwm / 8;
      cur_vr = -pwm;
      break;

    case 1: //Correct Right slight
      cur_vl = pwm;
      cur_vr = - pwm + pwm / 8;
      break;

    case 2: //Correct left
      cur_vl =  pwm - pwm / 6;
      cur_vr = -pwm;
      break;

    case 3: //Correct Right
      cur_vl = pwm;
      cur_vr = - pwm + pwm / 6;
      break;

    case 4: // Turn Left
      cur_vl = pwm / 8;
      cur_vr = -2 * pwm;
      break;

    case 5: //Turn Right
      cur_vl = 2 * pwm;
      cur_vr = -pwm / 8;
      break;

    case 6: // move backward
      cur_vl = -pwm;
      cur_vr = pwm;
      break;
  }
}

/*
  // determine a reward given the current state
  // NOTE: all distances in CM (ranger values)
  float get_reward() {
  //penalize if:
  //  1. front wall very close
  //  2. right wall very close
  //  3. right wall very far

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
*/

// determine a reward given the current state
// NOTE: all distances in CM (ranger values)
float get_reward() {
  /* penalize if:
    1. front wall very close
    2. right wall very close
    3. right wall very far
  */

  float min_f = 15.0;
  float min_r = 15.0;
  float max_r = 25.0;
  float target_r = 20.0;

  if (old_dist_f > dist_f) {                                  // moved forward

    if (dist_f > min_f) {                                         // there is still space in front
      if ((old_dist_r < max_r && dist_r > old_dist_r) ||           // veered in the correct direction +5
          (old_dist_r > target_r && dist_r < old_dist_r)) {

        if (dist_r < max_r && dist_r > min_r) {                            // ended within wall range +10
          return 5.0;
        }
        return 1.0;
      } else if ((old_dist_r > target_r && dist_r > old_dist_r) ||           // veered in the wrong direction -5
                 (old_dist_r < target_r && dist_r < old_dist_r)) {
        return -5;
      }

    } else if (dist_f <= min_f || old_dist_f <= min_f) {             // there is/was no space in front -5
      return -10.0;
    } else if (old_dist_f < dist_f && old_dist_f <= min_f) {     // moved backward when the old reading was too close +1
      return 5.0;
    }

    return 0;
  }
}

//////// ***** Q-learning functions END ***** ////////


/**********************************************************/
//////// ***** KEY AND PATROL FUNCTIONS START ***** ////////
/**********************************************************/

void check_key() {
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
      led.setColor(key_count, WHITE);
      led.show();
      // siren.tone(784, 50);
      // siren.tone(1046, 100);
    }
  }
}

void win()
{
  // optionally do stop motors dim the LED's etc.
  // Serial.print("stopped"); // or other warning
  Serial.println("YOU WIN!!");
  while (1) {
    led.setColor(0, RED);
    led.show();
    delay(100);
    led.setColor(0, GREEN);
    led.show();
    delay(100);
    led.setColor(0, BLUE);
    led.show();
    delay(100);
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

//////// ***** key and patrol functions END ***** ////////


/******************************************************************/
//////// ***** UTILITY, DEBBUGING/LOG FUNCTIONS START ***** ////////
/******************************************************************/

// limit the given value between the given min/max
float clamp(float xmin, float xx, float xmax) {
  if (xx < xmin) return xmin;
  if (xx > xmax) return xmax;
  return xx;
}


// choose a random number between 0 and 1
float rand_uniform() {
  return (float(rand()) / float(RAND_MAX)) * 1.0;
}


// save current q-table to Arduino's EEPROM memory
void write_EEPROM() {
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      EEPROM.update(i * NUM_ACTIONS + j, qtable[i][j]);
    }
  }
}


// load Arduino's EEPROM memory to q-table
void read_EEPROM() {
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      qtable[i][j] = EEPROM.read(i * NUM_ACTIONS + j);
    }
  }
}


// print current q-table
void print_qtable() {
  Serial.println("Q-TABLE");
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      Serial.print(qtable[i][j]);
      Serial.print('|');
    }
    Serial.println();
  }
  Serial.println();
}


// print cached qtable from EEPROM memory
void print_eeprom_state() {
  float value;
  Serial.println("EEPROM state");
  for (int i = 0; i < NUM_STATES; i++) {
    for (int j = 0; j < NUM_ACTIONS; j++) {
      EEPROM.get(i * NUM_ACTIONS + j, value);
      Serial.print(value);
      Serial.print('|');
    }
    Serial.println();
  }
  Serial.println();
}

// log q-learning activity
void log_rl_activity() {

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
}


// log key-finding activity
void log_key_activity() {

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

//////// ***** utility, debugging/log functions END ***** ////////


/******************************************************************/
//////// ***** MAIN PROGRAM START ***** ////////
/******************************************************************/

void setup() {
  Serial.begin(9600);

  // Hardware setup
  led.setpin(LED_PORT);
  led.setNumber(LED_NUM);
  led.setColor(OFF);
  led.show();

  siren.setpin(BUZZER_PORT);
  siren.noTone();

  // Read from eeprom or reset learning
  if (READ_EEPROM) {
    Serial.println("Reading EEPROM .....");
    read_EEPROM();
  } else {
    Serial.println("Initializing qtable .....");
    init_qtable();
  }
}

void loop() {

  // check patrol location
  patrol_detected = detect_patrol();
  if (patrol_detected) {
    MOTOR_R.setMotorPwm(0);
    MOTOR_L.setMotorPwm(0);
    siren.tone(523, 100);
    siren.tone(370, 100);
    delay(3000);
  }

  // cache qtable
  if (TICK % 500 == 0) {
    //Serial.println("CACHEING QTABLE ...");
    //write_EEPROM();
    //print_qtable();
    //print_eeprom_state();
  }

  // retrieve current distances
  delay(50);
  dist_f = ULTRA_F.distanceCm();
  dist_r = ULTRA_R.distanceCm();

  old_dist_r = dist_r;
  old_dist_f = dist_f;

  led.setColor(12, RED);
  led.show();

  // increment tick
  TICK += 1;

  // decrease 'randomness' in q-learning actions after 5000 tick
  if (TICK > 500) {
    EPS = 0.7;
    led.setColor(11, GREEN);
    led.show();
  }
  // remove 'randomness' in q-learning actions after 15000 tick
  if (TICK > 1000) {
    EPS = 0.9;
    led.setColor(10, BLUE);
    led.show();
  }

  // check for a key
  check_key();

  // find the best action to take next
  old_int_state = discretize_state();
  next_int_action = choose_action();
  realize_action(next_int_action);

  // safety mechanism for robot getting stuck on wall
  if (dist_f < 15) {

    cur_vl = -200.0;
    cur_vr = -100.0;
    reward = -5;
    MOTOR_L.setMotorPwm(cur_vl);
    MOTOR_R.setMotorPwm(cur_vr);
    delay(200);
  } else {

    // perform that action
    MOTOR_L.setMotorPwm(cur_vl);
    MOTOR_R.setMotorPwm(cur_vr);
    delay(ACTION_DELAY);

    // measure reward from previous action
    dist_f += ULTRA_F.distanceCm();
    dist_r += ULTRA_R.distanceCm();
    cur_int_state = discretize_state();
    reward = get_reward();
  }

  // update q table
  update_qtable();

  if (key_count == 8) {
    win();
  }

  //log_rl_activity();
  //log_key_activity();
}
