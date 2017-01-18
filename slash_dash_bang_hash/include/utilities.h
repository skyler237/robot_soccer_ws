# pragma once

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#define ROBOT_MAX_VXY 2.0
#define ROBOT_MAX_OMEGA 2*M_PI

static int sgn(double val);
static Vector3d saturateVelocity(Vector3d v);
static double angleMod(double angle);
static RobotState poseToRobotState(Pose2D robot);
static RobotState vectorToRobotState(Vector2d vec);
static Vector2d robotStateToVector(RobotState robot);
static BallState toBallState(Pose2D ball);
