#pragma once

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"
#include "slash_dash_bang_hash/State.h"
#include "controller/PID.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;


#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10

typedef boost::shared_ptr< ::slash_dash_bang_hash::State const> StateConstPtr;

class Controller {
public:
   Controller();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh;

  // Publishers and Subscribers
  ros::Publisher motor_pub_;
  ros::Subscriber desired_pose_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber game_state_sub_;
  soccerref::GameState gameState_;

  // PID controllers for each variable for each robot
  PID x_PID_;
  PID y_PID_;
  PID theta_PID_;

  slash_dash_bang_hash::State robot_state_;
  slash_dash_bang_hash::State desired_pose_;

  Vector3d command_;

  double max_xy_vel_;
  double max_omega_;

  void computeControl();
  void publishCommand();

  void desiredPoseCallback(const StateConstPtr &msg);
  void stateCallback(const StateConstPtr &msg);
  void gameStateCallback(const soccerref::GameState::ConstPtr &msg);
};
