#pragma once

#include <eigen3/Eigen/Eigen>
#include "slash_dash_bang_hash/State.h"

#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10
#define GOAL_BOX_WIDTH 0.619

using namespace Eigen;
using namespace std;
using namespace slash_dash_bang_hash;

class Skills
{
public:
  Skills();

  // ====================== General Skills =================================
  static State goToPoint(int robotId, State robot, Vector2d point);
  static void kick(string team, int robotId);

  // ====================== Offensive Skills =================================
  static State ballIntercept(int robotId, State robot, Vector2d ball);

  // ====================== Defensive Skills =================================
  static State followBallOnLine(int robotId, State robot, State ball, double x_pos);
  static State adaptiveRadiusGoalDefend(State robot_state, State ball_state);

  // ====================== Helper Functions =================================
  static Vector2d ballPredict(State ball, double time);

  // static Vector2d goal_;

};
