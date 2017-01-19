#pragma once

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"
#include "slash_dash_bang_hash/State.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

typedef boost::shared_ptr< ::slash_dash_bang_hash::State const> StateConstPtr;

#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10

// struct State
// {
//     // Positions
//     double x;
//     double y;
//     double theta;
//     // Velocities
//     double xdot;
//     double ydot;
//     double thetadot;
//     // Estimated states
//     double xhat;
//     double yhat;
//     double thetahat;
// };

// struct State
// {
//     // Positions
//     double x;
//     double y;
//     // Velocities
//     double xdot;
//     double ydot;
//     // Estimated states
//     double xhat;
//     double yhat;
// };

public class Estimator {
  Vector2d goal_;

  // Publishers and Subscribers
  ros::Subscriber ally1_vision_sub_, ally2_vision_sub_;
  ros::Subscriber opp1_vision_sub_, opp2_vision_sub_;
  ros::Subscriber ball_vision_sub_;

  ros::Publisher ally1_state_pub_, ally2_state_pub_;
  ros::Publisher opp1_state_pub_, opp2_state_pub_;
  ros::Publisher ball_state_pub_;

  ros::Subscriber game_state_sub_;
  ros::Subscriber ally1_state_sub_, ally2_state_sub_;
  ros::Subscriber opp1_state_sub_, opp2_state_sub_;
  ros::Subscriber ball_state_;
  soccerref::GameState gameState_;

  slash_dash_bang_hash::State ally1_vision_, ally2_vision_;
  slash_dash_bang_hash::State opp1_vision_, opp2_vision_;
  slash_dash_bang_hash::State ball_vision_;

  slash_dash_bang_hash::State ally1_state_, ally2_state_;
  slash_dash_bang_hash::State opp1_state_, opp2_state_;
  slash_dash_bang_hash::State ball_state_;

  void estimateStates();
  void publishStates();

  void visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg, const std::string& robot);
  void gameStateCallback(const soccerref::GameState::ConstPtr &msg);
}
