#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <iostream>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;
using namespace cv;

typedef boost::shared_ptr< ::slash_dash_bang_hash::State const> StateConstPtr;

#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10
#define GUI_NAME        "Soccer Overhead Camera"

#define FIELD_WIDTH_PIXELS      540.0 // measured from threshold of goal to goal
#define FIELD_HEIGHT_PIXELS     378.0 // measured from inside of wall to wall
#define CAMERA_WIDTH            640.0
#define CAMERA_HEIGHT           480.0

// These colours need to match the Gazebo materials
Scalar red[]    = {Scalar(0,   128, 128), Scalar(10,  255, 255)};
Scalar yellow[] = {Scalar(20,  128, 128), Scalar(30,  255, 255)};
Scalar green[]  = {Scalar(55,  128, 128), Scalar(65,  255, 255)};
Scalar blue[]   = {Scalar(115, 128, 128), Scalar(125, 255, 255)};
Scalar purple[] = {Scalar(145, 128, 128), Scalar(155, 255, 255)};


class Vision {
public:
   Vision();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh;
  Vector2d goal_;
  double tau_; // Dirty derivative gain

  image_transport::ImageTransport it(nh_);
  image_transport::Subscriber image_sub;

  // Handlers for vision position publishers
  ros::Publisher home1_pub;
  ros::Publisher home2_pub;
  ros::Publisher away1_pub;
  ros::Publisher away2_pub;
  ros::Publisher ball_pub;
  ros::Publisher ball_position_pub; // for publishing internally from the vision window

  // Use variables to store position of objects. These variables are very
  // useful when the ball cannot be seen, otherwise we'll get the position (0, 0)
  geometry_msgs::Pose2D poseHome1;
  geometry_msgs::Pose2D poseHome2;
  geometry_msgs::Pose2D poseAway1;
  geometry_msgs::Pose2D poseAway2;
  geometry_msgs::Pose2D poseBall;

  // void estimateStates();
  // void calculateVelocities();
  // void publishStates();
  //
  // void visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg);
  // void gameStateCallback(const soccerref::GameState::ConstPtr &msg);
  //
  // Helper functions
};