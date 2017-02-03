#pragma once

#include <eigen3/Eigen/Eigen>
#include "slash_dash_bang_hash/State.h"

#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10
#define BALL_RADIUS 0.022
#define GOAL_BOX_WIDTH 0.619

using namespace Eigen;
using namespace std;
using namespace slash_dash_bang_hash;

struct Zone_t {
  double min;
  double max;
};

class Skills
{
public:
  Skills();

  // ====================== General Skills =================================
  static State goToPoint(State robot, Vector2d point);
  static void kick(string team, int robotId);
  // Position behind ball facing towards "direction_point"
  static State getBall(State robot_state, State ball_state, Vector2d direction_point);

  // ====================== Offensive Skills =================================
  static double findBestShot(State ball_state, State ally_state, State opp1_state, State opp2_state);
  static State makeShot(string team, int robotId, State robot_state, State ball_state, Vector2d ball_destination);

  // ====================== Defensive Skills =================================
  static State followBallOnLine(int robotId, State robot, State ball, double x_pos);
  static State adaptiveRadiusGoalDefend(State robot_state, State ally_state, State ball_state);

  // ====================== Helper Functions =================================
  // Used for ball intercept/prediction
  static Vector2d ballPredict(State ball, double time);
  static Vector2d ballIntercept(State robot_state, State ball_state);
  static double getInterceptDifference(State robot_state, State ball_state, double time);
  // Used for finding shots
  static State mirrorState(State robot_state, int direction);
  static Zone_t updateOpenZone(Zone_t open_zone, Zone_t blocked_zone);
  static Zone_t findBlockedZone(State ball_state, State blocker_state);
  // Debug function
  static State hideInCorner(int corner_number);

  // static Vector2d goal_;

};
