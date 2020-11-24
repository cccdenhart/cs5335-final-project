#include <iostream>
#include <math.h>
#include <vector>

using std::cout;
using std::endl;

const float ALPHA = 0.2;
const float GAMMA = 0.9;

struct State {
    float pos_t;
    float dist_l;
    float dist_lf;
    float dist_f;
    float dist_rf;
    float dist_r;
};

struct Action {
    float vl;
    float vr;
};

struct State {
    vector<vector<float>> q_table;
} STATE;

// assumes 5 sensors with 4 positions each
// P1 > 2.0
// 1.0 <= P2 <= 2.0
// 0.5 <= P3 < 2.0
// P4 < 0.5
float reward_table[20][4] = {
      {0, -1, 0, -1},
      {-1, -1, 0, 0},
      {-1, -1, 0, 0},
      {-1, -1, -1, 0},
      {0, 0, 0, -1},
      {-1, 0, 0, 0},
      {-1, 0, 0, 0},
      {-1, 0, -1, 0},
      {0, 0, 0, -1},
      {-1, 0, 0, 0},
      {-1, 0, 0, 0},
      {-1, 0, -1, 0},
      {-1, 0, 0, -1},
      {-1, -1, 0, 0},
      {-1, -1, 1000,0},
      {-1, 0, -1, 0}
};

void update_qtable(vector<vector<float>> q_table,
                   Action action,
                   float reward,
                   State current_state,
                   State old_state) {
    float old_value = q_table[old_state][action];
    float max_q = max_element(q_table[current_state].begin(),
                              q_table[current_state].end());
    float new_value = (1 - ALPHA) * old_value + ALPHA * (reward + GAMMA * max_q);

    q_table[old_state][action] = new_value;
}

vector<vector<float>> init_qtable(int num_states, int num_actions) {
    vector<vector><float>> q_table;
    for (int i = 0; i != num_states; i++) {
        vector<float> row;
        for (int j = 0; j != num_actions; j++)
            row.push_back(0.0);
        q_table.push_back(row);
    }
    return q_table;
}


int main(int argc, char* argv[]) {
    cout << "starting algo" << endl;

    return 0;
}
