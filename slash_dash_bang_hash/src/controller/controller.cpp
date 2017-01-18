#include "slash_dash_bang_hash/controller.h"
#include "slash_dash_bang_hash/utilities.h"

#define CONTROL_TIME_STEP 0.0

Controller::Controller() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh private properly namespaces it.
  ros::NodeHandle priv_nh("~");

  // Set PID Gains
  double P, I, D, tau;
  tau = priv_nh.param<double>("tau", 0.05);
  P = priv_nh.param<double>("x_P", 5.0);
  I = priv_nh.param<double>("x_I", 0.0);
  D = priv_nh.param<double>("x_D", 0.0);
  x1_PID_.setGains(P, I, D, tau);
  x2_PID_.setGains(P, I, D, tau);

  P = priv_nh.param<double>("y_P", 5.0);
  I = priv_nh.param<double>("y_I", 0.0);
  D = priv_nh.param<double>("y_D", 0.0);
  y1_PID_.setGains(P, I, D, tau);
  y2_PID_.setGains(P, I, D, tau);

  P = priv_nh.param<double>("theta_P", 3.0);
  I = priv_nh.param<double>("theta_I", 0.0);
  D = priv_nh.param<double>("theta_D", 0.0);
  theta1_PID_.setGains(P, I, D, tau);
  theta2_PID_.setGains(P, I, D, tau);


  ally1_desired_pose_sub_ = nh.subscribe<slash_dash_bang_hash::RobotState>("ally1_desired_pose", 1, boost::bind(desiredPoseCallback, _1, "ally1"));
  ally2_desired_pose_sub_ = nh.subscribe<slash_dash_bang_hash::RobotState>("ally2_desired_pose", 1, boost::bind(desiredPoseCallback, _1, "ally2"));
  ally1_state_sub_ = nh.subscribe<slash_dash_bang_hash::RobotState>("ally1_state", 1, boost::bind(stateCallback, _1, "ally1"));
  ally2_state_sub_ = nh.subscribe<slash_dash_bang_hash::RobotState>("ally2_state", 1, boost::bind(stateCallback, _1, "ally2"));
  game_state_sub_ = nh.subscribe<soccerref::GameState>("/game_state", 1, gameStateCallback);

  motor_pub1_ = nh.advertise<geometry_msgs::Twist>("ally1/vel_cmds", 5);
  motor_pub2_ = nh.advertise<geometry_msgs::Twist>("ally2/vel_cmds", 5);

  // This is sort of ad-hoc (would be much better to be a parameter) but it works for now
  ally1_startingPos_.x = -0.5;
  ally1_startingPos_.y = -0.0;

  ally2_startingPos_.x = -1.0;
  ally2_startingPos_.y = -0.0;

}

void Controller::stateCallback(RobotStateConstPtr &msg, const std::string& robot)
{
    if(robot == "ally1")
        ally1_state_ = *msg;

    else if(robot == "ally2")
        ally2_state_ = *msg;
}

void Controller::desiredPoseCallback(RobotStateConstPtr &msg, const std::string& robot)
{
    int robotId = 0;
    if(robot == "ally1")
    {
      ally1_desired_pose_ = *msg;
      robotId = 1;
    }
    else if(robot == "ally2")
    {
      ally2_desired_pose_ = *msg;
      robotId = 2;
    }

    computeControl(robotId);
}

void Controller::computeControl(int robotId) {
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  double x1_command, x2_command;
  double y1_command, y2_command;
  double theta1_command, theta2_command;

  if (dt > CONTROL_TIME_STEP)
  {
    // Update ally1_command_ and ally2_command_ variables

    // Compute the PID values for each state variable
    if(robotId == 1)
    }
      x1_command = x1_PID_.computePID(ally1_state_.x, ally1_desired_pose_.x, dt);
      y1_command = y1_PID_.computePID(ally1_state_.y, ally1_desired_pose_.y, dt);
      theta1_command = theta1_PID_.computePID(ally1_state_.theta, ally1_desired_pose_.theta, dt);

      // TODO: We don't currently use theta to compute our command... would we like to change that?
      ally1_command_ << x1_command, y1_command;
    {
    else if(robotId == 2)
    {
      x2_command = x2_PID_.computePID(ally2_state_.x, ally2_desired_pose_.x, dt);
      y2_command = y2_PID_.computePID(ally2_state_.y, ally2_desired_pose_.y, dt);
      theta2_command = theta2_PID_.computePID(ally2_state_.theta, ally2_desired_pose_.theta, dt);

      // TODO: We don't currently use theta to compute our command... would we like to change that?
      ally2_command_ << x2_command, y2_command;
    }
    else {
      ROS_INFO("ERROR: Invalid Robot ID in controller::computeControl() function!");
    }
    
    // --- OR ---

    // // If we are able to reliable determine the velocity beforehand, we can use these functions:
    // x1_command = x1_PID_.computePIDDirect(ally1_state_.x, ally1_desired_pose_.x, ally1_state_.xdot, dt);
    // y1_command = y1_PID_.computePIDDirect(ally1_state_.y, ally1_desired_pose_.y, ally1_state_.ydot, dt);
    // theta1_command = theta1_PID_.computePIDDirect(ally1_state_.theta, ally1_desired_pose_.theta, ally1_state_.thetadot, dt);
    //
    // x2_command = x2_PID_.computePIDDirect(ally2_state_.x, ally2_desired_pose_.x, ally2_state_.xdot, dt);
    // y2_command = y2_PID_.computePIDDirect(ally2_state_.y, ally2_desired_pose_.y, ally2_state_.ydot, dt);
    // theta2_command = theta2_PID_.computePIDDirect(ally2_state_.theta, ally2_desired_pose_.theta, ally2_state_.thetadot, dt);

  }

  publishCommands(robotId);
}

void Controller::publishCommands(int robotId)
{
  switch (robotId) {
    case 1:
      moveRobot(1, ally1_command_);
      break;

    case 2:
      moveRobot(2, ally2_command_);
      break;

    default:
      ROS_INFO("ERROR: Invalid Robot ID in controller::publishCommands() function!");
      break;
  }


}

void Controller::moveRobot(int robotId, Vector3d vel_world)
{
    geometry_msgs::Twist vel;
    vel.linear.x = vel_world(0);
    vel.linear.y = vel_world(1);
    vel.angular.z = vel_world(2);

    if(robotId == 1)
        motor_pub1_.publish(vel);
    else if(robotId == 2)
        motor_pub2_.publish(vel);
}




void Controller::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
    gameState = *msg;
}

int main(int argc, char **argv)
{
    param_init();
    ros::init(argc, argv, "home");
    ros::NodeHandle nh;


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
