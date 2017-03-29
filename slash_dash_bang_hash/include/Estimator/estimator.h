#pragma once

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"
#include "slash_dash_bang_hash/State.h"
#include "slash_dash_bang_hash/Pose2DStamped.h"
#include <string.h>

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

typedef boost::shared_ptr< ::slash_dash_bang_hash::State const> StateConstPtr;
typedef boost::shared_ptr< ::slash_dash_bang_hash::Pose2DStamped const> Pose2DStampedConstPtr;

#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10

#define MAX_QUEUE_SIZE 200

using namespace slash_dash_bang_hash;

class Estimator {
public:
  Estimator();
  void estimateStates();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh;
  string team_;
  Vector2d goal_;

  // Publishers and Subscribers
  ros::Subscriber vision_data_sub_;
  ros::Publisher state_pub_;

  ros::Subscriber game_state_sub_;
  ros::Subscriber state_sub_;


  soccerref::GameState gameState_;
  slash_dash_bang_hash::State vision_data_;
  std_msgs::Header vision_header_;
  slash_dash_bang_hash::State state_;  
  slash_dash_bang_hash::State state_prev_;

  bool new_vision_data_;
  double sample_period_;
  double LPF_corner_freq_xy_;
  double LPF_alpha_xy_;
  double LPF_corner_freq_theta_;
  double LPF_alpha_theta_;
  double xy_vel_damping_coeff_;
  double theta_vel_damping_coeff_;
  double tau_;

  // Sample queue
  State sampleQ_[MAX_QUEUE_SIZE]; // Array of States to keep track of samples
  int sampleQ_size_;
  int sampleQ_index_;
  int sampleQ_cnt_;
  void addSample(State sample);
  State updateSamples(State update, int samples_old, double dt);

  // Estimator implementations
  void LPF_Estimator();
  void predictAndCorrectEstimator();

  State correctStateWithMeasurementsOnly(State measurement, State predicted_state);
  State correctState(State prediction, State measurement, double dt);
  State predictState(State state, double dt);

  State lowPassFilterStates(State prev_state, State measurement, double alpha_xy, double alpha_theta);
  void calculateVelocities();
  void publishStates();



  void visionCallback(const Pose2DStampedConstPtr &msg);
  void gameStateCallback(const soccerref::GameState::ConstPtr &msg);

  // Helper functions
  double lowPassFilter(double alpha, double previous, double measured);
  slash_dash_bang_hash::State poseToState(geometry_msgs::Pose2D pose);
};
