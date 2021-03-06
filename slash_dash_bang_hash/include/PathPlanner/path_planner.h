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
using namespace slash_dash_bang_hash;

typedef boost::shared_ptr< ::slash_dash_bang_hash::State const> StateConstPtr;

#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10


class PathPlanner
{
public:
   PathPlanner();

 private:
   // Node handles, publishers, subscribers
   ros::NodeHandle nh_;
   ros::NodeHandle priv_nh;

  Vector2d goal_;
  double max_xy_vel_;
  double time_step_;
  int robot_number_;

  // Publishers and Subscribers
  ros::Publisher desired_pose_pub_;

  ros::Subscriber destination_sub_;

  ros::Subscriber game_state_sub_;
  ros::Subscriber ally1_state_sub_, ally2_state_sub_;
  ros::Subscriber opp1_state_sub_, opp2_state_sub_;
  ros::Subscriber ball_state_sub_;
  soccerref::GameState gameState_;

  slash_dash_bang_hash::State destination_; // End goal
  slash_dash_bang_hash::State desired_pose_; // Next step to get there
  slash_dash_bang_hash::State ally1_state_, ally2_state_;
  slash_dash_bang_hash::State opp1_state_, opp2_state_;
  slash_dash_bang_hash::State ball_state_;

  void planPath();
  void publishDesiredPose();
  State simpleCurvedPathToDestination(State robot_state, State destination);
  State dribbleBallToDestination(State robot_state, State ally_state, State destination);
  State avoidBall(State destination);
  State avoidRobot(bool isAlly, State other_robot_state, State destination);
  State avoidAlly(State destination, State robot_state, State ally_state);
  State avoidOpponent(State destination, State robot_state, State opponent_state);
  State avoidObject(State destination, State robot_state, State object_state, double object_radius, double avoidance_margin);

  void stateCallback(const StateConstPtr &msg, const std::string& robot);
  void destinationCallback(const StateConstPtr &msg);
  void gameStateCallback(const soccerref::GameState::ConstPtr &msg);
};
