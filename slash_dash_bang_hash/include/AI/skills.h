#pragma once

#include <eigen3/Eigen/Eigen>

using namespace Eigen;
using namespace std;
using namespace slash_dash_bang_hash;

struct BallState
{
    // Positions
    double x;
    double y;
    // Velocities
    double xdot;
    double ydot;
    // Estimated states
    double xhat;
    double yhat;
};


public class Skills
{
  // ====================== Offensive Skills =================================
  RobotState ballIntercept(int robotId, RobotState robot, Vector2d ball);
  RobotState goToPoint(int robotId, RobotState robot, Vector2d point);

  // ====================== Defensive Skills =================================
  RobotState followBallOnLine(int robotId, RobotState robot, Vector2d ball, double x_pos);

  // ====================== Helper Functions =================================
  Vector2d ballPredict(BallState ball, double time);

}
