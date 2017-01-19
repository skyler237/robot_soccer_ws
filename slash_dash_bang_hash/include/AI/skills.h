#pragma once

#include <eigen3/Eigen/Eigen>
#include "slash_dash_bang_hash/State.h"

using namespace Eigen;
using namespace std;
using namespace slash_dash_bang_hash;

public class Skills
{
  // ====================== Offensive Skills =================================
  State ballIntercept(int robotId, State robot, Vector2d ball);
  State goToPoint(int robotId, State robot, Vector2d point);

  // ====================== Defensive Skills =================================
  State followBallOnLine(int robotId, State robot, Vector2d ball, double x_pos);

  // ====================== Helper Functions =================================
  Vector2d ballPredict(State ball, double time);

}
