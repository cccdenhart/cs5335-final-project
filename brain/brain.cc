
#include <iostream>
#include <string>
#include <filesystem>
#include <math.h>
#include <string.h>
#include <cstdlib>

#include "robot.hh"

using std::vector;
using std::cout;
using std::endl;

const float ALPHA = 0.2;
const float GAMMA = 0.9;
const float BASE_VEL = 2.0;
const float MIN_VEL = -3.0;
const float MAX_VEL = 3.0;
const int NUM_STATES = 10;
const int NUM_ACTIONS = 8;
float EPS = 0.3;
float TICK = 0;

// a raw representation of a robot state
struct QState {
	float dist;
};

// a raw representation of a robot action
struct QAction {
	float vl;
	float vr;
};

// a representation of the world state
struct WorldState {
	vector<vector<float>> qtable;
};

WorldState STATE;


// converts a raw state representation to an integer representation
// (usable by the Q-Table)
int discretize_state(QState state) {
	float max_dist = 3.0;
  float dist = clamp(0.0, state.dist, max_dist);
	int norm = floor(dist / max_dist * NUM_STATES);
	return norm;
}

// converts a raw action representation to an integer representation
// (usable by the Q-Table)
int discretize_action(QAction action) {
	float diff = clamp(MIN_VEL, action.vl - action.vr, MAX_VEL);
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

// initializes the q-table
vector<vector<float>> init_qtable(int num_states, int num_actions) {
	vector<vector<float>> qtable;
	for (int i = 0; i != num_states; i++) {
			vector<float> row;
			for (int j = 0; j != num_actions; j++)
					row.push_back(0.0);
			qtable.push_back(row);
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
		vector<vector<float>> qtable,
		QState state,
		float eps = EPS
) {

	int int_state = discretize_state(state);
	int int_action;
	float rand_val = rand_uniform();

	if (rand_val < eps) {
		vector<float> possible_actions = qtable[int_state];
		int_action = std::distance(possible_actions.begin(), 
																	 std::max_element(possible_actions.begin(), possible_actions.end()));
	}
	else {
		int_action = floor(rand_uniform() * NUM_ACTIONS);
	}
	
	return int_action;
}

// update the qtable with new information
vector<vector<float>> update_qtable(
		vector<vector<float>> qtable,
		int action,
		float reward,
		int current_state,
		int old_state,
		float learning_rate = ALPHA,
		float reward_decay = GAMMA
) {
	float old_value = qtable[old_state][action];
	float max_q = *max_element(qtable[current_state].begin(),
								 						 qtable[current_state].end());
	float new_value = (1 - learning_rate) * old_value + learning_rate * (reward + reward_decay * max_q);

	qtable[old_state][action] = new_value;

	return qtable;
}

// determine a reward given the current state
float get_reward(QState state) {
	if (state.dist < 0.6)
		return -5.0;
	else if (state.dist < 1.0)
		return 0.0;
	else if (state.dist < 2.0)
		return 5.0;
	else if (state.dist < 2.5)
		return 1.0;
	else
		return -5.0;
}

// steps to perform on each 'tick'
void callback(Robot* robot) {
	// float speed = 6 * clamp(0.0, robot->get_range() - 0.25, 1.0);
	
	// increment tick
	TICK += 1;
	if (TICK > 5000)
		EPS = 0.9;
	
	// find the best action to take next
	QState old_state = { robot->get_range() };
	int old_int_state = discretize_state(old_state);
	int next_int_action = choose_action(STATE.qtable, old_state);
	QAction next_action = realize_action(next_int_action);

	// perform that action	
	robot->set_vel(next_action.vl, next_action.vr);

	// measure reward from previous action
	QState cur_state = { robot->get_range() };
	int cur_int_state = discretize_state(cur_state);
	float reward = get_reward(cur_state);

	// update q table
	STATE.qtable = update_qtable(STATE.qtable, next_int_action, reward, cur_int_state, old_int_state);

	cout << "tick: " << TICK << endl;
	cout << "state: " << cur_int_state << ", " << cur_state.dist << endl;
	cout << "action: " 
			 << next_int_action 
			 << ", " 
			 << "(" << next_action.vl << ", " << next_action.vr << ")" 
			 << endl;
	cout << "reward: " << reward << endl;
	cout << "=====\n" << endl;
}

// initialize the world
int main(int argc, char* argv[]) {
	Robot* robot = 0;

	std::string bname(basename(argv[0]));
	cout << "bin: [" << bname << "]" << endl;

	if (bname == "gz_brain") {
			cout << "making robot: Gazebo mode" << endl;
			robot = new GzRobot(argc, argv, callback);
	}

	if (bname == "rg_brain") {
			cout << "making robot: Ranger mode" << endl;
			robot = new RgRobot(argc, argv, callback);
	}

	STATE.qtable = init_qtable(NUM_STATES, NUM_ACTIONS);

	robot->do_stuff();

	delete robot;
	return 0;
}
