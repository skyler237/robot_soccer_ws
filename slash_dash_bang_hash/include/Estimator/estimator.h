#pragma once

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"
#include "slash_dash_bang_hash/State.h"
#include <string.h>

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

typedef boost::shared_ptr< ::slash_dash_bang_hash::State const> StateConstPtr;

#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10

class Estimator {
public:
   Estimator();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh;
  string team_;
  Vector2d goal_;
  double tau_; // Dirty derivative gain

  // Publishers and Subscribers
  ros::Subscriber vision_data_sub_;
  ros::Publisher state_pub_;

  ros::Subscriber game_state_sub_;
  ros::Subscriber state_sub_;


  soccerref::GameState gameState_;
  slash_dash_bang_hash::State vision_data_;
  slash_dash_bang_hash::State state_;
  slash_dash_bang_hash::State state_prev_;

  double sample_period_;
  double LPF_corner_freq_xy_;
  double LPF_alpha_xy_;
  double LPF_corner_freq_theta_;
  double LPF_alpha_theta_;

  void estimateStates();
  void lowPassFilterStates();
  void calculateVelocities();
  void publishStates();

  void visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg);
  void gameStateCallback(const soccerref::GameState::ConstPtr &msg);

  // Helper functions
  double lowPassFilter(double alpha, double previous, double measured);
  slash_dash_bang_hash::State poseToState(geometry_msgs::Pose2D pose);
};
