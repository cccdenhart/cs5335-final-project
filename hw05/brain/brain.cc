
#include <iostream>
#include <math.h>
#include <vector>

#include "robot.hh"

using std::cout;
using std::endl;

// States that determine the behavior based on sensors and state
#define FIND_WALL 0 // moving in a particular direction
#define FOLLOW_WALL 1 // following a wall
#define RESOLVE_CORNER 2 // turn around a corner

// Structure to keep track of the state of the robot
class RobotState {
  public:
    int row;    // keeps track of the row the robot is in
    int state;  // keeps track of the function state (e.g. FOLLOW_WALL)
    char dir;   // keeps track of the goal direction
    bool search; // true when the robot reaches row 10

    RobotState() {
      row = 0;
      dir = 'E';
      state = FIND_WALL;
    }
    

    // returns the actual direction of the robot. The var determines the
    // variability. E.g. var = 0 would return 'N' only if the robot pos_t
    // is exaclty 0. var = 0.5 would return 'N' for -0.1 < pos_t < 0.1.
    // Anything else returns 'T' 
    char direction(double pos_t, double var) {
      if (pos_t < 0 + var && pos_t > 0 - var)
        return 'N';
      if (pos_t < 1.57 + var && pos_t > 1.57 - var)
        return 'W';
      if (pos_t < -1.57 + var && pos_t > -1.57 - var)
        return 'E';
      if (pos_t < 3.14 - var && pos_t > -3.14 + var)
        return 'T';
      
      return 'S';
    }
};

RobotState state;


// BASIC DIRECTION FUNCTIONS

void turnTo(Robot* robot, double r) {
  if (robot->pos_t > r) {
    robot->set_vel(1.5, -1.5);
  } else if (robot->pos_t < r) {
    robot->set_vel(-1.5, 1.5);
  }
}

void faceDir(Robot* robot, char dir) {
  switch(dir) {
    case('N'):
      turnTo(robot, 0);
      break;
    case('S'):
      turnTo(robot, 3.14);
      break;
    case('E'):
      turnTo(robot, -1.57);
      break;
    case('W'):
      turnTo(robot, 1.57);
      break;
  }
}


// STATE FUNCTIONS

// Follows a straigh path in the direction of state.dir
void findWall(Robot* robot) {
  if (state.direction(robot->pos_t, 0.03) != state.dir) {
    faceDir(robot, state.dir);
    return;
  } else {
    if (robot->range < 1.2) {
      if (state.row == 10) {
        if (state.dir == 'E') {
          state.dir = 'W';
          return;
        }

        if (state.dir == 'W') {
          state.search = true;
          state.state = FOLLOW_WALL;
          state.dir = 'E';
          return;
        }
      } 

      state.state = FOLLOW_WALL;
      state.dir = 'W';
      return;
    }
  }

  robot->set_vel(2.0, 2.0);
}

// Keeps close to and follows a given wall in the direction of state.dir
void followWall(Robot* robot) {

  // If the wall 'disappears', assume it is a corner
  if (robot->range > 2) {
    if (state.direction(robot->pos_t, 0.75) == 'W' 
        || state.direction(robot->pos_t, 0.75) == 'N' || state.search) {
      state.state = RESOLVE_CORNER;
    } else {
      state.state = FIND_WALL;
      state.dir = 'E';
    }
  }

  // If the wall gets too far, compensate to get it in range again
  if (robot->range >= 1.5) {
    if (state.dir == 'E')
      robot->set_vel(0.8, 2.0);
    if (state.dir == 'W')
      robot->set_vel(2.0, 0.8);
    return;
  }

  // If the wall is within range follow a straight path
  if (robot->range > 1.0) {
    robot->set_vel(2.0, 2.0);
    return;
  }
  

  // If the wall gets too close, compansate to get it in range again
  if (robot->range < 1.0) {
    if (state.dir == 'E')
      robot->set_vel(1.5, -0.8);
    if (state.dir == 'W')
      robot->set_vel(-0.8, 1.5);
  }
}

void resolveCorner(Robot* robot) {
  if (robot->range > 2) {
    if (state.dir == 'E')
      robot->set_vel(0.5, 2.0);
    if (state.dir == 'W')
      robot->set_vel(2.0, 0.5);
    return;
  }

  if (robot->range <= 1.6) {
    if (state.direction(robot->pos_t, 0.75) == 'N' || 
        state.direction(robot->pos_t, 0.75) == 'E')
      state.row += 1;
    if (state.direction(robot->pos_t, 0.75) == 'S')
      state.row -= 1;
    
    state.state = FIND_WALL;
    state.dir = 'E';
    if (state.row == 11)
      state.dir = 'W';
  }
}


void callback(Robot* robot)
{ 
  /*
  cout << "*******************" << endl;
  cout << "STATE:    " << state.state << endl;
  cout << "Goal dir: " << state.dir << endl;
  cout << "Curr dir: " << state.direction(robot->pos_t, 0.75) << endl;
  cout << "pos_t:    " << robot->pos_t << endl;
  cout << "range:    " << robot->range << endl;
  cout << "Row:      " << state.row << endl;
  cout << "*******************" << endl;
  cout << endl;
  */

}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
