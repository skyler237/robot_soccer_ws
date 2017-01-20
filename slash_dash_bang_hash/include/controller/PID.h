#pragma once

class PID
{
  // Constructors
public:
  PID();
  PID(double p, double i, double d, double tau);

  double kp_;
  double ki_;
  double kd_;
  double integrator_;
  double differentiator_;
  double last_error_;
  double last_state_;
  double tau_;

  double computePID(double current, double desired, double dt);
  // Use this when the derivative of the state is already known
  double computePIDDirect(double x, double x_r, double xdot, double dt);
  void setGains(double p, double i, double d, double tau);
};
