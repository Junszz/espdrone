#ifndef ESPDRONE_PID_H
#define ESPDRONE_PID_H

#include <ros/node_handle.h>

namespace espdrone_controller{

class PID
{
public:
  struct parameters {
    parameters();
    bool enabled;
    double time_constant;
    double k_p;
    double k_i;
    double k_d;
    double limit_i;
    double limit_output;
  } parameters_;

  struct state {
    state();
    double p, i, d;
    double input, dinput;
    double dx;
  } state_;

public:
  PID();
  PID(const parameters& parameters);
  ~PID();

  void init(const ros::NodeHandle &param_nh);
  void reset();

  double update(double input, double x, double dx, double dt);
  double update(double error, double dx, double dt);

  double getFilteredControlError(double& filtered_error, double time_constant, double dt);
};
}

#endif // ESPDRONE_PID_H
