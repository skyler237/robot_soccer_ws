#pragma once

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"
#include "slash_dash_bang_hash/RobotState.h"
#include "slash_dash_bang_hash/PID.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;


#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10

typedef boost::shared_ptr< ::slash_dash_bang_hash::RobotState const> RobotStateConstPtr;

public class Controller {
  // Publishers and Subscribers
  ros::Publisher motor_pub1_;
  ros::Publisher motor_pub2_;
  ros::Subscriber ally1_goal_sub_, ally2_goal_sub_;
  ros::Subscriber ally1_state_sub_, ally2_state_sub_;
  ros::Subscriber game_state_sub_;
  soccerref::GameState gameState_;

  // PID controllers for each variable for each robot
  PID x1_PID_, x2_PID_;
  PID y1_PID_, y2_PID_;
  PID theta1_PID_, theta2_PID_;

  slash_dash_bang_hash::RobotState ally1_state_, ally2_state_;
  slash_dash_bang_hash::RobotState ally1_goal_, ally2_goal_;

  Vector3d ally1_command_;
  Vector3d ally2_command_;

  void computeControl();
  void publishCommands();

  void moveRobot(int robotId, Vector3d v_world);
  void goalCallback(RobotStateConstPtr &msg, const std::string& robot);
  void stateCallback(RobotStateConstPtr &msg, const std::string& robot);
  void gameStateCallback(const soccerref::GameState::ConstPtr &msg);
}
