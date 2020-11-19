
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

#define FACE_EAST  0
#define CENTER     1

int state = 0;

double clamp(double x0, double x, double x1);
/*{
  if (x < x0) return x0;
  if (x > x1) return x1;
  return x;
}*/

double 
sign(double x)
{
  if (x < 0) {
    return -1.0;
  }
  else {
    return 1.0;
  }
}

void
callback(Robot* robot)
{
    double vel;
    // - Face east
    // - Move forward or backwards until range = 1.5

    switch (state) {
      case FACE_EAST:
        vel = sign(robot->pos_t) * clamp(0.75, abs(robot->pos_t), 2.5);
        robot->set_vel(vel, -vel);
        if (abs(robot->pos_t) < 0.05) {
          state = CENTER;
        }
        break;
      case CENTER:
        //cout << "range: " << robot->range << "; theta: " << robot->pos_t << endl;
        vel = clamp(-2.5, 1.5 - robot->range, 2.5);
        vel = sign(vel) * clamp(0.75, abs(vel), 2.5);
        robot->set_vel(vel, vel);
        break;
      default:
        break;
    }
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    state = FACE_EAST;

    return 0;
}
