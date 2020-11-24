#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <assert.h>
#include <stdio.h>
#include <ctype.h>
#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;

#include "robot.hh"

int port;

RgRobot::RgRobot(int argc, char* argv[], void (*cb)(Robot*))
    : on_update(cb)
{
    int rv;
    port = open("/dev/ttyUSB0", O_RDWR);
    assert(port > 0);

    struct termios cfg;
    rv = tcgetattr(port, &cfg);
    assert(rv != -1);

    // https://github.com/todbot/arduino-serial/blob/master/arduino-serial-lib.c
    cfsetispeed(&cfg, B9600);
    cfsetospeed(&cfg, B9600);
    cfg.c_cflag &= ~PARENB;
    cfg.c_cflag &= ~CSTOPB;
    cfg.c_cflag &= ~CRTSCTS;
    cfg.c_cflag |= CS8;

    rv = tcsetattr(port, TCSANOW, &cfg);
    assert(rv != -1);
}

RgRobot::~RgRobot()
{

}

float
RgRobot::get_range()
{
    return range;
}

void
RgRobot::set_vel(double lvel, double rvel)
{
    lvel = clamp(-4, lvel, 4);
    rvel = clamp(-4, rvel, 4);

    char temp[100];
    int s0 = (int)round(lvel * 50);
    int s1 = (int)round(rvel * 50);

    snprintf(temp, 96, "%d %d\n", s0, s1);
    write(port, temp, strlen(temp));
    //cout << "cmd: " << temp << endl;
}

static
string
serial_read_line()
{
    string temp("");
    char cc = 0;

    while (1) {
        read(port, &cc, 1);

        if (cc == '\n' && temp.size() > 0) {
            return temp;
        }

        if (cc == '\n' || temp.size() > 30) {
            temp.clear();
        }

        if (isdigit(cc) || cc == ' ') {
            temp.push_back(cc);
        }
    }
}

void
RgRobot::read_range()
{
    string line = serial_read_line();
    const char* temp = line.c_str();

    cout << "[" << temp << "]" << endl;

    int rg, sL, sR;
    int rv = sscanf(temp, "%d %d %d", &rg, &sL, &sR);
    if (rv != 3) {
        this->range = 0;
        return;
    }

    this->range = rg / 100.0f;
    cout << "range = " << this->range << "; "
         << "speeds = " << sL << "," << sR << endl;
}

void
RgRobot::do_stuff()
{
    while (1) {
        cout << "\n == iterate ==" << endl;
        this->read_range();
        this->on_update(this);
    }
}
