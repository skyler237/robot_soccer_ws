#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include <image_transport/image_transport.h>
#include <stdio.h>
#include <stdint.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "slash_dash_bang_hash/State.h"
#include "slash_dash_bang_hash/Pose2DStamped.h"
#include <geometry_msgs/Twist.h>


#include <cmath>
#include <math.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;
using namespace cv;

typedef boost::shared_ptr< ::slash_dash_bang_hash::State const> StateConstPtr;
typedef boost::shared_ptr< ::geometry_msgs::Twist const> VelConstPtr;

#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10
#define GUI_NAME        "Soccer Overhead Camera"

#define FIELD_WIDTH_PIXELS      540.0 // measured from threshold of goal to goal
#define FIELD_HEIGHT_PIXELS     378.0 // measured from inside of wall to wall
#define CAMERA_WIDTH            640.0
#define CAMERA_HEIGHT           480.0


class Vision {
public:
   Vision();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh;
  Vector2d goal_;
  double tau_; // Dirty derivative gain

  image_transport::Subscriber image_sub;

  // Handlers for vision position publishers
  ros::Publisher home1_pub;
  ros::Publisher home2_pub;
  ros::Publisher away1_pub;
  ros::Publisher away2_pub;
  ros::Publisher ball_pub;
  ros::Publisher referee_ball_pub; // Need to publish Pose2D for the referee
  ros::Publisher ball_position_pub; // for publishing internally from the vision window

  //subscriber to the desired pose
  ros::Subscriber desired_pose_sub_;
  ros::Subscriber destination_sub_;
  ros::Subscriber vel_command_sub_;


  //desired pose
  slash_dash_bang_hash::State desired_pose_;
  //destination, end goal
  slash_dash_bang_hash::State destination_;
  geometry_msgs::Twist command_;

  // Used to store image time stamps
  std_msgs::Header img_header_;


  // Use variables to store position of objects. These variables are very
  // useful when the ball cannot be seen, otherwise we'll get the position (0, 0)
  geometry_msgs::Pose2D poseHome1;
  geometry_msgs::Pose2D poseHome2;
  geometry_msgs::Pose2D poseAway1;
  geometry_msgs::Pose2D poseAway2;
  geometry_msgs::Pose2D poseBall;


  enum robot_color { red, green, blue, yellow, purple, pink };

  std::string home1_color_str_;
  std::string home2_color_str_;
  std::string away1_color_str_;
  std::string away2_color_str_;
  robot_color home1_color_;
  robot_color home2_color_;
  robot_color away1_color_;
  robot_color away2_color_;



  //Vision functions
  Rect findYellowRobot(Mat img);
  bool isInYellowRange(int hue, int sat);
  void getRobotPose(Mat img);
  void visionCallback(const sensor_msgs::ImageConstPtr& msg);
  Rect crop(Mat img);
  Mat smoothing(Mat img, int radius);
  void findWhiteBall(Mat img);
  bool isInwhiteRange(int hue, int sat, int val);
  bool isInPinkRange(int hue, int sat, int val);
  void findPinkBall(Mat img);
  void drawPosDest(Mat img);
  void setDesiredPose(const StateConstPtr &msg);
  void setDestination(const StateConstPtr &msg);
  static Point2d getCenterOfMass(Moments moment);
  static bool compareMomentAreas(Moments moment1, Moments moment2);
  int findLongestLine(vector<Vec4f> lines);
  Point2d imageToWorldCoordinates(Point2d point_i, Mat img);
  Mat thresholdImage(Mat img, robot_color robotColor);
  Vector3d findCenterRobot(Mat img, robot_color robotColor);
  Vector3d convertToWorldCoord(Vector3d pixelCoord, int cols, int rows);
  Point convertWorldToPixel(double world_x, double world_y, int cols, int rows);
  robot_color getColorFromString(std::string str);
  void setVelCommand(const VelConstPtr &msg);
  void createTrackbars(Mat img);





  //
  // Helper functions
};
