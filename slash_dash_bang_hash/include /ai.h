#pragma once

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;


#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10

struct RobotState
{
    // Positions
    double x;
    double y;
    double theta;
    // Velocities
    double xdot;
    double ydot;
    double thetadot;
    // Estimated states
    double xhat;
    double yhat;
    double thetahat;
};

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

public class AI {
  string team_;
  Vector2d goal_;

  // Publishers and Subscribers
  ros::Publisher motor_pub1_;
  ros::Publisher motor_pub2_;
  ros::Subscriber vsub_ally1_, vsub_ally2_;
  ros::Subscriber vsub_opp1_, vsub_opp2_;
  ros::Subscriber vsub_ball_;
  ros::Subscriber game_state_sub_;
  soccerref::GameState gameState_;

  RobotState ally1_, ally2_;
  RobotState opp1_, opp2_;
  BallState ball_;
  RobotState ally1_startingPos_;
  RobotState ally2_startingPos_;

  void param_init();
  void computeGoal();

  void moveRobot(int robotId, Vector3d v_world);
  void skill_followBallOnLine(RobotState robot, Vector2d ball, double x_pos, int robotId);
  void skill_goToPoint(RobotState robot, Vector2d point, int robotId);
  void play_rushGoal(RobotState robot, Vector2d ball, int robotId);
  void visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg, const std::string& robot);
  void gameStateCallback(const soccerref::GameState::ConstPtr &msg);
}
