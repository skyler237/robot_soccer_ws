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


using namespace slash_dash_bang_hash;

class AI {
public:
   AI();

 private:
  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh;
  string team_;
  int robot_number_;
  Vector2d goal_;

  // Publishers and Subscribers

  ros::Publisher ally1_destination_pub_;
  ros::Publisher ally2_destination_pub_;
  // ros::Publisher ally1_kick_pub_;
  // ros::Publisher ally2_kick_pub_;

  ros::Subscriber ally1_state_sub_, ally2_state_sub_;
  ros::Subscriber opp1_state_sub_, opp2_state_sub_;
  ros::Subscriber ball_state_sub_;
  ros::Subscriber game_state_sub_;

  soccerref::GameState gameState_;

  State ally1_state_, ally2_state_;
  State ally1_destination_, ally2_destination_;
  State opp1_state_, opp2_state_;
  State ball_state_;

  int manual_destinations_;

  Vector2d ally1_startingPos_;
  Vector2d ally2_startingPos_;

  void param_init();
  void computeDestination();
  void publishDestinations();

  void checkForKick(int roboId);

  State play_basicDefense(State robot, State ball);
  State play_standardOffense();
  State play_rushGoal(State robot, State ball);
  State play_findBestShot(int robotId, State robot, State ball);
  State play_skillsTournament(State robot_state);
  State play_playsTournament(State robot_state);
  State play_onePlayer(State robot_state);
  void stateCallback(const StateConstPtr &msg, const std::string& robot);
  void gameStateCallback(const soccerref::GameState::ConstPtr &msg);
};
