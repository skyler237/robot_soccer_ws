#include "Vision/Vision.h"
#include "Utilities/utilities.h"


Vision::Vision() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");
  tau_ = priv_nh.param<double>("dirty_deriv_gain", 0.05);

  // Subscribe to camera
  image_sub = it.subscribe("/camera1/image_raw", 1, imageCallback);

  home1_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/home1", 5);
  home2_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/home2", 5);
  away1_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/away1", 5);
  away2_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/away2", 5);
  ball_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/ball", 5);


}

void Vision::visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    vision_data_ = poseToState(*msg);

    estimateStates();
}

void Vision::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
    gameState_ = *msg;
}

void Vision::estimateStates()
{


  // TODO: Actually implement an Vision here -- currently just passes the data through
  state_ = vision_data_;


  calculateVelocities();
  publishStates();
}

void Vision::publishStates()
{
  state_prev_ = state_;

  state_pub_.publish(state_);
}

void Vision::calculateVelocities()
{
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  if (dt > 0.0)
  {
    state_.xdot = (2.0*tau_ - dt)/(2.0*tau_ + dt)*state_.xdot + 2.0/(2.0*tau_ + dt)*(state_.x - state_prev_.x);
    state_.ydot = (2.0*tau_ - dt)/(2.0*tau_ + dt)*state_.ydot + 2.0/(2.0*tau_ + dt)*(state_.y - state_prev_.y);
  }
}

slash_dash_bang_hash::State Vision::poseToState(geometry_msgs::Pose2D pose)
{
  slash_dash_bang_hash::State state;

  // Flip coordinates if team is away or if we've swapped sides
  if((team_ == "away") ^ gameState_.second_half)
  {
      state.x = -pose.x;
      state.y = -pose.y;
      state.theta = fmod((pose.theta + 180.0), 360.0);
      //  state.theta = pose.theta;
  }
  else{
    state.x = pose.x;
    state.y = pose.y;
    state.theta = pose.theta;
  }

  return state;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "home");
    ros::NodeHandle nh_;

    Vision Vision_node;

    ros::spin();
    // ros::Rate loop_rate(30);
    // while(ros::ok())
    // {
    //     // process any callbacks
    //     ros::spinOnce();
    //
    //     // force looping at a constant rate
    //     loop_rate.sleep();
    // }
}
