#include "Estimator/estimator.h"
#include "Utilities/utilities.h"


Estimator::Estimator() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");
  priv_nh.param<string>("team", team_, "home");
  tau_ = priv_nh.param<double>("dirty_deriv_gain", 0.05);
  sample_period_ = priv_nh.param<double>("sample_period", 0.01); // Default 100 Hz
  LPF_corner_freq_xy_ = priv_nh.param<double>("LPF_corner_freq_xy", 10); // Default 10 Hz
  LPF_alpha_xy_ = exp(-1.0*LPF_corner_freq_xy_*sample_period_);
  LPF_corner_freq_theta_ = priv_nh.param<double>("LPF_corner_freq_theta", 10); // Default 10 Hz
  LPF_alpha_theta_ = exp(-1.0*LPF_corner_freq_theta_*sample_period_);

  game_state_sub_ = nh_.subscribe<soccerref::GameState>("/game_state", 1, &Estimator::gameStateCallback, this);
  vision_data_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("vision_data", 1, &Estimator::visionCallback, this);

  state_pub_ = nh_.advertise<slash_dash_bang_hash::State>("state", 5);

  ROS_INFO("Initializing estimator");


}

void Estimator::visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    vision_data_ = poseToState(*msg);
    ROS_INFO("Estimator callback");

    estimateStates();
}

void Estimator::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
    gameState_ = *msg;
}

void Estimator::estimateStates()
{


  // TODO: Actually implement an estimator here -- currently just passes the data through
  ROS_INFO("Estimator estimateStates");
  state_ = vision_data_;
  lowPassFilterStates();
  calculateVelocities();

  publishStates();
}

void Estimator::publishStates()
{
  state_prev_ = state_;
  ROS_INFO("Estimator publishStates");
  state_pub_.publish(state_);
}

void Estimator::calculateVelocities()
{
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  // if (dt > 0.0) -- time not working for now
  // {
    state_.xdot = tustinDerivative(state_.xhat, state_prev_.xhat, state_prev_.xdot, tau_, dt);
    state_.ydot = tustinDerivative(state_.yhat, state_prev_.yhat, state_prev_.ydot, tau_, dt);
  // }
}

double Estimator::lowPassFilter(double alpha, double previous, double measured)
{
  return alpha*previous  + (1.0 - alpha)*measured;
}

void Estimator::lowPassFilterStates()
{
  state_.xhat = lowPassFilter(LPF_alpha_xy_, state_prev_.xhat, vision_data_.x);
  state_.yhat = lowPassFilter(LPF_alpha_xy_, state_prev_.yhat, vision_data_.y);
  state_.thetahat = lowPassFilter(LPF_alpha_theta_, state_prev_.thetahat, vision_data_.theta);
}

slash_dash_bang_hash::State Estimator::poseToState(geometry_msgs::Pose2D pose)
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

    Estimator estimator_node;

    ROS_INFO("Main for estimator");

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
