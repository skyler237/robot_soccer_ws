#include "Estimator/estimator.h"
#include "Utilities/utilities.h"
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>


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
  LPF_corner_freq_xy_ = priv_nh.param<double>("LPF_corner_freq_xy", 100); // Default 10 Hz
  LPF_alpha_xy_ = priv_nh.param<double>("LPF_alpha_xy", exp(-1.0*LPF_corner_freq_xy_*sample_period_));
  LPF_corner_freq_theta_ = priv_nh.param<double>("LPF_corner_freq_theta", 100); // Default 10 Hz
  LPF_alpha_theta_ = priv_nh.param<double>("LPF_alpha_theta", exp(-1.0*LPF_corner_freq_theta_*sample_period_));
  xy_vel_damping_coeff_ = priv_nh.param<double>("xy_vel_damping_coeff", 0.99);
  theta_vel_damping_coeff_ = priv_nh.param<double>("theta_vel_damping_coeff", 0.99);
  sampleQ_size_ = priv_nh.param<int>("sample_queue_size", 100);
  sampleQ_cnt_ = 0;
  sampleQ_index_ = 0;

  // Initialize the state off the field
  state_.x = 5;
  state_.y = 5;
  state_.theta = 0;
  state_.xhat = 5;
  state_.yhat = 5;
  state_.thetahat = 0;




  game_state_sub_ = nh_.subscribe<soccerref::GameState>("/game_state", 10, &Estimator::gameStateCallback, this);
  vision_data_sub_ = nh_.subscribe<slash_dash_bang_hash::Pose2DStamped>("vision_data", 10, &Estimator::visionCallback, this);

  state_pub_ = nh_.advertise<slash_dash_bang_hash::State>("state", 5);

  // ROS_INFO("Initializing estimator");


}

void Estimator::visionCallback(const Pose2DStampedConstPtr &msg)
{
    double now = ros::Time::now().toSec();
    static double prev = 0;
    double dt = now - prev;
    prev = now;

    // ROS_INFO("estimator visionCallback: dt=%f", dt);

    vision_header_ = msg->header;
    // ROS_INFO("Estimator: vision stamp: secs=%d, nsecs=%d", vision_header_.stamp.sec, vision_header_.stamp.nsec);
    // ROS_INFO("Vision lag = %f", now - vision_header_.stamp.toSec());
    vision_data_ = poseToState(msg->pose);


    new_vision_data_ = true;

}

void Estimator::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
    double now = ros::Time::now().toSec();
    static double prev = 0;
    double dt = now - prev;
    prev = now;

    // ROS_INFO("gameStateCallback: dt=%f", dt);
    gameState_ = *msg;
}

void Estimator::estimateStates()
{
  // predictAndCorrectEstimator();
  LPF_Estimator();


  publishStates();
}

void Estimator::LPF_Estimator() {
  state_ = vision_data_;
  state_ = lowPassFilterStates(state_prev_, vision_data_, LPF_alpha_xy_, LPF_alpha_theta_);
  calculateVelocities();
}

void Estimator::predictAndCorrectEstimator() {
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  // ROS_INFO("estimateStates: dt=%f", dt);

  if(new_vision_data_) {
    // Process the measurement
    new_vision_data_ = false;

    // Measure the lag
    double vision_lag = now - vision_header_.stamp.toSec();
    double lag_sample_periods = vision_lag/sample_period_;

    // Round to the nearest integer
    int samples_old_; // Number of actual samples
    if(fmod(lag_sample_periods, 1.0) >= 0.5) {
      samples_old_ = (int) (lag_sample_periods + 1);
    }
    else {
      samples_old_ = (int) lag_sample_periods;
    }

    // ROS_INFO("Samples old: %d, sampleQ_cnt: %d", samples_old_, sampleQ_cnt_);
    // Go back and update from the right sample
    if(samples_old_ < sampleQ_cnt_) {
      state_ = updateSamples(vision_data_, samples_old_, dt);
      // state_ = lowPassFilterStates(state_prev_, vision_data_, LPF_alpha_xy_, LPF_alpha_theta_);
    }
  }
  else {
    // Predict states
    state_ = predictState(state_, dt);
    // Store predictions in a queue to be updated
    addSample(state_);

  }
}



State Estimator::predictState(State state,  double dt)
{
  State pred_state;

  // Project based on velocity
  pred_state.xhat = state.xhat + state.xdot*dt;
  pred_state.yhat = state.yhat + state.ydot*dt;
  pred_state.thetahat = state.thetahat + state.thetadot*dt;

  // Account for velocity damping
  pred_state.xdot = xy_vel_damping_coeff_*state.xdot;
  pred_state.ydot = xy_vel_damping_coeff_*state.ydot;
  pred_state.thetadot = theta_vel_damping_coeff_*state.thetadot;

  return pred_state;
}

State Estimator::correctState(State prediction, State measurement, double dt)
{
  State corrected_state = prediction;

  // Update velocities
  corrected_state.xdot = tustinDerivative(measurement.x, prediction.xhat, prediction.xdot, tau_, dt);
  corrected_state.ydot = tustinDerivative(measurement.y, prediction.yhat, prediction.ydot, tau_, dt);
  corrected_state.thetadot = tustinDerivative(measurement.theta, prediction.thetahat, prediction.thetadot, tau_, dt);

  // Re-predict states
  corrected_state = predictState(corrected_state, dt);

  return corrected_state;
}

State Estimator::correctStateWithMeasurementsOnly(State measurement, State predicted_state)
{
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  static State prev_measurement = measurement;
  State corrected_state = lowPassFilterStates(state_prev_, vision_data_, LPF_alpha_xy_, LPF_alpha_theta_);


  // Update velocities
  corrected_state.xdot = tustinDerivative(measurement.x, prev_measurement.x, prev_measurement.xdot, tau_, dt);
  corrected_state.ydot = tustinDerivative(measurement.y, prev_measurement.y, prev_measurement.ydot, tau_, dt);
  corrected_state.thetadot = tustinDerivative(measurement.theta, prev_measurement.theta, prev_measurement.thetadot, tau_, dt);
   // ROS_INFO("measurement: x=%f, y=%f, theta=%f", measurement.x, measurement.y, measurement.theta);
   // ROS_INFO("prev_measurement: x=%f, y=%f, theta=%f", prev_measurement.x, prev_measurement.y, prev_measurement.theta);
   // ROS_INFO("prev_measurement vel: x=%f, y=%f, theta=%f", prev_measurement.xdot, prev_measurement.ydot, prev_measurement.thetadot);
   // ROS_INFO("corrected_state: x=%f, y=%f, theta=%f", corrected_state.xhat, corrected_state.yhat, corrected_state.thetahat);
   // ROS_INFO("corrected_state vel: x=%f, y=%f, theta=%f", corrected_state.xdot, corrected_state.ydot, corrected_state.thetadot);

  // Re-predict states
  corrected_state = predictState(corrected_state, dt);
  prev_measurement = measurement;
  prev_measurement.xdot = corrected_state.xdot;
  prev_measurement.ydot = corrected_state.ydot;
  prev_measurement.thetadot = corrected_state.thetadot;

  return corrected_state;
}

void Estimator::publishStates()
{
  state_prev_ = state_;

  // Transfer estimated states to the states used by the rest of the architecture
  State state_to_publish = state_;
  state_to_publish.x = state_.xhat;
  state_to_publish.y = state_.yhat;
  state_to_publish.theta = state_.thetahat;

  state_pub_.publish(state_to_publish);
}

void Estimator::calculateVelocities()
{
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  dt = sample_period_;

  // if (dt > 0.0) -- time not working for now
  // {
    state_.xdot = tustinDerivative(state_.xhat, state_prev_.xhat, state_prev_.xdot, tau_, dt);
    state_.ydot = tustinDerivative(state_.yhat, state_prev_.yhat, state_prev_.ydot, tau_, dt);
    state_.thetadot = tustinDerivative(state_.thetahat, state_prev_.thetahat, state_prev_.thetadot, tau_, dt);

    // state_.xdot = tustinDerivative(state_.x, state_prev_.x, state_prev_.xdot, tau_, dt);
    // state_.ydot = tustinDerivative(state_.y, state_prev_.y, state_prev_.ydot, tau_, dt);
    // state_.thetadot = tustinDerivative(state_.theta, state_prev_.theta, state_prev_.thetadot, tau_, dt);
  // }
}

double Estimator::lowPassFilter(double alpha, double previous, double measured)
{
  return alpha*previous  + (1.0 - alpha)*measured;
}

State Estimator::lowPassFilterStates(State prev_state, State measurement, double alpha_xy, double alpha_theta)
{
  State result = prev_state;
  result.xhat = lowPassFilter(alpha_xy, prev_state.xhat, measurement.x);
  result.yhat = lowPassFilter(alpha_xy, prev_state.yhat, measurement.y);
  result.thetahat = lowPassFilter(alpha_theta, prev_state.thetahat, measurement.theta);
  return result;
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

//==================================================================
//======================= Samples Queue code =======================
//==================================================================

void Estimator::addSample(State sample) {
  sampleQ_[sampleQ_index_] = sample;
  sampleQ_index_ = (sampleQ_index_ + 1)%sampleQ_size_; // update index and wrap around
  sampleQ_cnt_ = saturate(sampleQ_cnt_ + 1, 0, sampleQ_size_); // Saturate sample count at the size of the queue
}

State Estimator::updateSamples(State update, int samples_old_, double dt) {
  samples_old_ = saturate(samples_old_, 0, sampleQ_cnt_); // Don't try to update past the data we have stored
  int update_index = ((sampleQ_index_ - samples_old_) + sampleQ_size_) % sampleQ_size_; // Go back the right number of samples, but with wrap around


  // Correct state here
  sampleQ_[update_index] = correctStateWithMeasurementsOnly(update, sampleQ_[update_index]);


  int curr_index = update_index;
  int next_index;
  for(int i = 0; i < samples_old_; i++) {
    next_index = (curr_index + 1)%sampleQ_size_; // Get next index with wrap around
    sampleQ_[next_index] = predictState(sampleQ_[curr_index], dt);
    curr_index = next_index;
  }
  return sampleQ_[curr_index];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "home");
    ros::NodeHandle nh_;

    Estimator estimator_node;

    // ROS_INFO("Main for estimator");

    // Initialize state_prev_

    // ros::spin();
    ros::Rate loop_rate(100); // Run at 100 Hz
    while(ros::ok())
    {
        // process any callbacks
        ros::spinOnce();

        estimator_node.estimateStates();

        // force looping at a constant rate
        loop_rate.sleep();
    }
}
