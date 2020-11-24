#ifndef ROBOT_HH
#define ROBOT_HH

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

class Robot {
  public:
    virtual ~Robot() = 0;
    virtual float get_range() = 0;
    virtual void set_vel(double lvel, double rvel) = 0;
    virtual void do_stuff() = 0;
};

class GzRobot : public Robot {
  public:
    void (*on_update)(Robot*);

    float pos_x;
    float pos_y;
    float range;
    float pos_t;

    GzRobot(int argc, char* argv[], void (*cb)(Robot*));
    ~GzRobot();

    bool at_goal();
    void done();
    void on_scan(ConstSonarStampedPtr &msg);
    void on_pose(ConstPoseStampedPtr &msg);

    float get_range();
    void set_vel(double lvel, double rvel);
    void do_stuff();


  private:
    bool task_done;

    gazebo::transport::NodePtr node;
    gazebo::transport::PublisherPtr vel_pub;
    gazebo::transport::SubscriberPtr scan_sub;
    gazebo::transport::SubscriberPtr pose_sub;
};

class RgRobot : public Robot {
  public:
    RgRobot(int argc, char* argv[], void (*cb)(Robot*));
    ~RgRobot();

    void read_range();

    float get_range();
    void set_vel(double lvel, double rvel);
    void do_stuff();

    void (*on_update)(Robot*);
    float range;
};

static
double
clamp(double xmin, double xx, double xmax)
{
    if (xx < xmin) return xmin;
    if (xx > xmax) return xmax;
    return xx;
}

#endif
