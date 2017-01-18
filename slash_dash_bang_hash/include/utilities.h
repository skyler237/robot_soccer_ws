# pragma once

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#define ROBOT_MAX_VXY 2.0
#define ROBOT_MAX_OMEGA 2*M_PI

int sgn(double val);
Vector3d saturateVelocity(Vector3d v);
double angleMod(double angle);
RobotState toRobotState(Pose2D robot);
BallState toBallState(Pose2D ball);
