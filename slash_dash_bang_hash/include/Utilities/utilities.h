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
using namespace slash_dash_bang_hash;


#define ROBOT_MAX_VXY 2.0
#define ROBOT_MAX_OMEGA 2*M_PI
#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10
#define KICKER_WIDTH 0.04


#define NOMINAL_VEL 1.5

#define KICKING_RANGE (0.03 + ROBOT_RADIUS)
#define POSSESSION_RANGE (0.1 + ROBOT_RADIUS)

#define BALL_RADIUS 0.022
#define AVOIDANCE_MARGIN 0.045
#define ROBOT_AVOIDANCE_MARGIN 0.1

#define MINIMUN_SHOT_ON_GOAL_DISTANCE 1.5
#define GOAL_X (FIELD_WIDTH/2)
#define GOAL_Y 0.0
#define SHOT_Y_MARGIN 0.05 // 5 cm

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

  // Returns whether the object is within a box of given width and length in front of the robot
  Vector2d getVecPerpendicularTo(Vector2d vec);

  double vectorProjectedDistance(Vector2d vec, Vector2d reference);
  Vector2d vectorProjection(Vector2d vec, Vector2d reference);
  Vector2d rotateVector(Vector2d vec, double theta); // Performs right-handed rotation on a 2D vector

  bool isInFront(State robot, State object, double box_radius, double box_length);

  bool isObjectBetween(State object, double object_radius, State robot1, State robot2);

  bool ballIsInPossessionOf(State robot, State ball);

  // Returns true if the robot is close enough and in position to make a goal at the given y
  bool readyForGoalShot(double shot_destination_y, State robot_state, State ally_state, State opp1_state, State opp2_state, State ball_state);
#endif /* end of include guard:  */



// };
