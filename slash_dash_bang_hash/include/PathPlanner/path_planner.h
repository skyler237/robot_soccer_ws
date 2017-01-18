#pragma once

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"
#include "slash_dash_bang_hash/RobotState.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

typedef boost::shared_ptr< ::slash_dash_bang_hash::RobotState const> RobotStateConstPtr;

#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10

// struct RobotState
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

public class PathPlanner {
  Vector2d goal_;

  // Publishers and Subscribers
  ros::Publisher ally1_desired_pose_pub_;
  ros::Publisher ally2_desired_pose_pub_;

  ros::Subscriber game_state_sub_;
  ros::Subscriber ally1_state_sub_, ally2_state_sub_;
  ros::Subscriber opp1_state_sub_, opp2_state_sub_;
  ros::Subscriber ball_state_;
  soccerref::GameState gameState_;

  slash_dash_bang_hash::RobotState ally1_destination_, ally2_destination_; // End goal
  slash_dash_bang_hash::RobotState ally1_desired_pose_, ally2_desired_pose_; // Next step to get there
  slash_dash_bang_hash::RobotState ally1_state_, ally2_state_;
  slash_dash_bang_hash::RobotState opp1_state_, opp2_state_;
  BallState ball_state_;

  void planPath();


  void destinationCallback(RobotStateConstPtr &msg, const std::string& robot);
  void gameStateCallback(const soccerref::GameState::ConstPtr &msg);
}
