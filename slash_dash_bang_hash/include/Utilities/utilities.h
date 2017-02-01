#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdio.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "slash_dash_bang_hash/State.h"

using namespace Eigen;
using namespace std;


#define ROBOT_MAX_VXY 2.0
#define ROBOT_MAX_OMEGA 2*M_PI

// class Utilities
// {
// public:
  int sgn(double val);
  Vector3d saturateVelocity(Vector3d v);
  double saturate(double x, double min, double max);
  double angleMod(double angle);
  static slash_dash_bang_hash::State poseToState(geometry_msgs::Pose2D robot);
  slash_dash_bang_hash::State vectorToState(Vector2d vec);
  slash_dash_bang_hash::State vectorToState(Vector2d vec);
  Vector2d stateToVector(slash_dash_bang_hash::State robot);
  void printVector(Vector2d vector, string name);

#endif /* end of include guard:  */



// };
