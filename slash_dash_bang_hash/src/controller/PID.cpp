#include "controller/PID.h"
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>

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
  if(dt == 0.0 || fabs(error) > 9999999)
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
    differentiator_ = (2.0*tau_ - dt)/(2.0*tau_ + dt)*differentiator_ + 2.0/(2.0*tau_ + dt)*(error - last_error_);
  }

  last_error_ = error;
  last_state_ = current;

  // ROS_INFO("PID: error=%f, integrator=%f, differentiator=%f", error, integrator_, differentiator_);

  return kp_*error + ki_*integrator_ - kd_*differentiator_;
}

// This can be used if the derivative of the state is already known and does not need ot be numerically calculated
double PID::computePIDDirect(double error, double xdot, double dt)
{
  // Handle border cases (don't compute anything)
  if(dt == 0.0 || fabs(error) > 9999999)
  {
    last_error_ = error;
    return 0.0;
  }

  double stop_threshold = 0.05;
  double integrate_threshold = 0.2;

  // Numerical integration -- only apply when we are close to the target
  if(fabs(error) > stop_threshold && fabs(error) < integrate_threshold) {
    integrator_ += dt/2*(error + last_error_);
  }
  else {
    integrator_ = 0.0;
  }

  last_error_ = error;

  // ROS_INFO("PIDDirect: error=%f, integrator=%f, xdot=%f", error, integrator_, xdot);


  // ROS_INFO("Control values: P=%f, I=%f, D=%f", kp_*error, ki_*integrator_, -kd_*xdot);

  if(fabs(error) < stop_threshold) {
    return 0.0;
  }
  else {
    return kp_*error + ki_*integrator_ - kd_*xdot;
  }
}

void PID::setGains(double p, double i, double d, double tau)
{
  kp_ = p;
  ki_ = i;
  kd_ = d;
  tau_ = tau;
}
