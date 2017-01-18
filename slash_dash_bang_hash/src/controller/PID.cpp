#include "PID.h"

PID::PID()
{
  kp_ = 0.0;
  ki_ = 0.0;
  kd_ = 0.0;
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
  tau_ = 0.0;
}

PID::PID(double p, double i, double d, double tau) :
  kp_(p),ki_(i),kd_(d),tau_(tau)
{
  integrator_ = 0.0;
  differentiator_ = 0.0;
  last_error_ = 0.0;
  last_state_ = 0.0;
}

double PID::computePID(double current, double desired, double dt)
{
  double error = desired - current;

  // Handle border cases (don't compute anything)
  if(dt == 0.0 || std::abs(error) > 9999999)
  {
    last_error_ = error;
    last_state_ = current;
    return 0.0;
  }

  // Numerical integration
  integrator_ += dt/2*(error + last_error_);

  // Numerical derivative (From Dr. Beard's UAV book Chapter 6)
  if(dt > 0.0)
  {
    // Should I use (current - last_state_) at the end??
    differentiator_ = (2.0*tau_ - dt)/(2.0*tau_ + dt)*differentiator_ + 2.0/(2.0*tau_ + dt)*(error_ - last_error_);
  }

  last_error_ = error;
  last_state_ = current;

  return kp_*error + ki_*integrator_ - kd_*differentiator_;
}

// This can be used if the derivative of the state is already known and does not need ot be numerically calculated
double PID::computePIDDirect(double x, double x_r, double xdot, double dt)
{
  double error = x_r - x;

  // Handle border cases (don't compute anything)
  if(dt == 0.0 || std::abs(error) > 9999999)
  {
    last_error_ = error;
    last_state_ = current;
    return 0.0;
  }

  // Numerical integration
  integrator_ += dt/2*(error + last_error_);

  last_error_ = error;
  last_state_ = current;

  return kp_*error + ki_*integrator_ - kd_*xdot;
}

void PID::setGains(double p, double i, double d, double tau)
{
  kp_ = p;
  ki_ = i;
  kd_ = d;
  tau_ = tau;
}