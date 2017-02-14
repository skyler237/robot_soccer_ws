#pragma once

#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "slash_dash_bang_hash/MotorSpeeds.h"
#include "slash_dash_bang_hash/State.h"
#include "controller/PID.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;
using namespace slash_dash_bang_hash;


typedef boost::shared_ptr< ::slash_dash_bang_hash::State const> StateConstPtr;

// Model parameters
#define RHO 1 // Wheel radius
#define SX1 0 // Wheel spin vectors - body frame
#define SY1 0
#define SX2 0
#define SY2 0
#define SX3 0
#define SY3 0
#define RX1 0 // Wheel position vectors - body frame
#define RY1 0
#define RX2 0
#define RY2 0
#define RX3 0
#define RY3 0

class MotionControl {
public:
   MotionControl();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh;

  // Publishers and Subscribers
  ros::Publisher motor_speed_pub_;
  ros::Subscriber robot_state_sub_;
  ros::Subscriber velocity_sub_;

  geometry_msgs::Twist desired_velocity_;
  MotorSpeeds motor_speeds_;
  State robot_state_;

  Matrix3d M_; // Kinematics matrix

  void computeMotorSpeeds();
  void publishSpeeds();

  void velocityCallback(const TwistConstPtr &msg);
  void robotStateCallback(const StateConstPtr &msg);
};
