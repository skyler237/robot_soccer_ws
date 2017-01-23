#include "controller/controller.h"
#include "Utilities/utilities.h"

#define CONTROL_TIME_STEP 0.0

Controller::Controller() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");

  // Set PID Gains
  double P, I, D, tau;
  tau = priv_nh.param<double>("tau", 0.05);
  P = priv_nh.param<double>("x_P", 3.0);
  I = priv_nh.param<double>("x_I", 0.0);
  D = priv_nh.param<double>("x_D", 0.0);
  x1_PID_.setGains(P, I, D, tau);
  x2_PID_.setGains(P, I, D, tau);

  P = priv_nh.param<double>("y_P", 3.0);
  I = priv_nh.param<double>("y_I", 0.0);
  D = priv_nh.param<double>("y_D", 0.0);
  y1_PID_.setGains(P, I, D, tau);
  y2_PID_.setGains(P, I, D, tau);

  P = priv_nh.param<double>("theta_P", 3.0);
  I = priv_nh.param<double>("theta_I", 0.0);
  D = priv_nh.param<double>("theta_D", 0.0);
  theta1_PID_.setGains(P, I, D, tau);
  theta2_PID_.setGains(P, I, D, tau);

  max_xy_vel_ = priv_nh.param<double>("max_xy_vel", 10.0);
  max_omega_ = priv_nh.param<double>("max_omega", 45.0);


  ally1_desired_pose_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally1_desired_pose", 1, boost::bind(&Controller::desiredPoseCallback, this, _1, "ally1"));
  ally2_desired_pose_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally2_desired_pose", 1, boost::bind(&Controller::desiredPoseCallback, this, _1, "ally2"));
  ally1_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally1_state", 1, boost::bind(&Controller::stateCallback, this, _1, "ally1"));
  ally2_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally2_state", 1, boost::bind(&Controller::stateCallback, this, _1, "ally2"));
  game_state_sub_ = nh_.subscribe<soccerref::GameState>("/game_state", 1, &Controller::gameStateCallback, this);

  motor_pub1_ = nh_.advertise<geometry_msgs::Twist>("ally1/vel_cmds", 1);
  motor_pub2_ = nh_.advertise<geometry_msgs::Twist>("ally2/vel_cmds", 1);
  //ROS_INFO("Init");

}

void Controller::stateCallback(const StateConstPtr &msg, const std::string& robot)
{
  //ROS_INFO("Controller state call back");
    int robotId = 0;
    if(robot == "ally1") {
      ally1_state_ = *msg;
      robotId = 1;
    }
    else if(robot == "ally2") {
        ally2_state_ = *msg;
        robotId = 2;
    }

    computeControl(robotId);
}

void Controller::desiredPoseCallback(const StateConstPtr &msg, const std::string& robot)
{
  //ROS_INFO("C desiredPoseCallback");


    if(robot == "ally1")
    {
      ally1_desired_pose_ = *msg;
    }
    else if(robot == "ally2")
    {
      ally2_desired_pose_ = *msg;
    }

}

void Controller::computeControl(int robotId) {
  //ROS_INFO("C computeControl");

  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  double x1_command, x2_command;
  double y1_command, y2_command;
  double theta1_command, theta2_command;

  if (dt > CONTROL_TIME_STEP && (gameState_.play || gameState_.reset_field))
  {
    // Update ally1_command_ and ally2_command_ variables

    // Compute the PID values for each state variable
    if(robotId == 1)
    {
      x1_command = saturate(x1_PID_.computePID(ally1_state_.x, ally1_desired_pose_.x, dt), -1*max_xy_vel_, max_xy_vel_);
      y1_command = saturate(y1_PID_.computePID(ally1_state_.y, ally1_desired_pose_.y, dt), -1*max_xy_vel_, max_xy_vel_);
      theta1_command = saturate(theta1_PID_.computePID(ally1_state_.theta, ally1_desired_pose_.theta, dt), -1*max_omega_, max_omega_);

      // TODO: We don't currently use theta to compute our command... would we like to change that?
      ally1_command_ << x1_command, y1_command, theta1_command;
      // ROS_INFO("Robot 1 Control: x_vel=%f, y_vel=%f, omega=%f", x1_command, y1_command, theta1_command);
    }
    else if(robotId == 2)
    {
      x2_command = saturate(x2_PID_.computePID(ally2_state_.x, ally2_desired_pose_.x, dt), -1*max_xy_vel_, max_xy_vel_);
      y2_command = saturate(y2_PID_.computePID(ally2_state_.y, ally2_desired_pose_.y, dt), -1*max_xy_vel_, max_xy_vel_);
      theta2_command = saturate(theta2_PID_.computePID(ally2_state_.theta, ally2_desired_pose_.theta, dt), -1*max_omega_, max_omega_);

      // TODO: We don't currently use theta to compute our command... would we like to change that?
      ally2_command_ << x2_command, y2_command, theta2_command;
      // ROS_INFO("Robot 2 Control: x_vel=%f, y_vel=%f, omega=%f", x2_command, y2_command, theta2_command);
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
    publishCommand(robotId);
  }
  else {
    x1_command = 0;
    y1_command = 0;
    theta1_command = 0;

    x2_command = 0;
    y2_command = 0;
    theta2_command = 0;
    publishCommand(1);
    publishCommand(2);
  }




}

// ================ Original Demoteam implementation of control ================
// void skill_goToPoint(RobotPose robot, Vector2d point, int robotId)
// {
//     Vector2d dirPoint = point - robot.pos;
//     Vector2d vxy = dirPoint * CONTROL_K_XY;
//
//     // control angle to face the goal
//     Vector2d dirGoal = goal - robot.pos;
//     double theta_d = atan2(dirGoal(1), dirGoal(0));
//     double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d);
//
//     // Output velocities to motors
//     Vector3d v;
//     v << vxy, omega;
//     v = utility_saturateVelocity(v);
//     moveRobot(v, robotId);
// }

void Controller::publishCommand(int robotId)
{
  //ROS_INFO("C publishCommand");

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
  //ROS_INFO("C moveRobot");

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
  //ROS_INFO("C gameStateCallback");

    gameState_ = *msg;
}

int main(int argc, char **argv)
{
//  ROS_INFO("C main");

    ros::init(argc, argv, "home");
    ros::NodeHandle nh_;

    Controller controller_node;

    ros::spin();
    //ROS_INFO("C main 2");

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
