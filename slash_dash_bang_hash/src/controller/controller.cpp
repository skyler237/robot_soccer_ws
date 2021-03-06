#include "controller/controller.h"
#include "Utilities/utilities.h"

#define CONTROL_TIME_STEP 0.01

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
  P = priv_nh.param<double>("x_P", 3.5);
  I = priv_nh.param<double>("x_I", 0.0);
  D = priv_nh.param<double>("x_D", 0.0);
  x_PID_.setGains(P, I, D, tau);

  P = priv_nh.param<double>("y_P", 3.5);
  I = priv_nh.param<double>("y_I", 0.0);
  D = priv_nh.param<double>("y_D", 0.0);
  y_PID_.setGains(P, I, D, tau);

  P = priv_nh.param<double>("theta_P", 3.5);
  I = priv_nh.param<double>("theta_I", 0.0);
  D = priv_nh.param<double>("theta_D", 0.0);
  theta_PID_.setGains(P, I, D, tau);

  max_xy_vel_ = priv_nh.param<double>("max_xy_vel", 10.0);
  max_omega_ = priv_nh.param<double>("max_omega", 180.0);


  desired_pose_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("desired_pose", 1, &Controller::desiredPoseCallback, this);
  state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("robot_state", 1, &Controller::stateCallback, this);
  game_state_sub_ = nh_.subscribe<soccerref::GameState>("/game_state", 1, &Controller::gameStateCallback, this);

  motor_pub_ = nh_.advertise<geometry_msgs::Twist>("vel_command", 1);

}

void Controller::stateCallback(const StateConstPtr &msg)
{
    robot_state_ = *msg;

    computeControl();
}

void Controller::desiredPoseCallback(const StateConstPtr &msg)
{
  desired_pose_ = *msg;

  // desired_pose_.theta = fmod((desired_pose_.theta + 180.0), 360.0);
}

void Controller::computeControl() {

  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;


  double x_command;
  double y_command;
  double theta_command;

  // if (dt > CONTROL_TIME_STEP && (gameState_.play || gameState_.reset_field))
  if ((gameState_.play || gameState_.reset_field))
  {

    // Update ally_command_ and ally2_command_ variables

    // Compute the PID values for each state variable

    // Correct the "theta gap" -- choose the direction that is quickest
    double theta_error = desired_pose_.theta - robot_state_.theta;
    if(fabs(theta_error) > fabs(360.0 - theta_error)) {
      theta_error = theta_error - 360.0;
    }
    else if(fabs(theta_error) > fabs(360.0 + theta_error)) {
      theta_error = theta_error + 360.0;
    }

    // x_command = saturate(x_PID_.computePID(robot_state_.x, desired_pose_.x, dt), -1*max_xy_vel_, max_xy_vel_);
    // y_command = saturate(y_PID_.computePID(robot_state_.y, desired_pose_.y, dt), -1*max_xy_vel_, max_xy_vel_);
    // theta_command = saturate(theta_PID_.computePID(robot_state_.theta, desired_pose_.theta, dt), -1*max_omega_, max_omega_);

    // HACK!
    x_command = saturate(x_PID_.computePIDDirect(desired_pose_.x - robot_state_.x, robot_state_.xdot, dt), -1*max_xy_vel_, max_xy_vel_);
    y_command = saturate(y_PID_.computePIDDirect(desired_pose_.y - robot_state_.y, robot_state_.ydot, dt), -1*max_xy_vel_, max_xy_vel_);
    theta_command = saturate(theta_PID_.computePIDDirect(theta_error, robot_state_.thetadot, dt), -1*max_omega_, max_omega_);

    command_ << x_command, y_command, theta_command;

    // --- OR ---

    // // If we are able to reliable determine the velocity beforehand, we can use these functions:
    // x_command = x_PID_.computePIDDirect(robot_state_.x, desired_pose_.x, robot_state_.xdot, dt);
    // y_command = y_PID_.computePIDDirect(robot_state_.y, desired_pose_.y, robot_state_.ydot, dt);
    // theta_command = theta_PID_.computePIDDirect(robot_state_.theta, desired_pose_.theta, robot_state_.thetadot, dt);
    //
    // x2_command = x2_PID_.computePIDDirect(ally2_state_.x, ally2_desired_pose_.x, ally2_state_.xdot, dt);
    // y2_command = y2_PID_.computePIDDirect(ally2_state_.y, ally2_desired_pose_.y, ally2_state_.ydot, dt);
    // theta2_command = theta2_PID_.computePIDDirect(ally2_state_.theta, ally2_desired_pose_.theta, ally2_state_.thetadot, dt);


    publishCommand();
  }
  else {
    //
    x_command = 0;
    y_command = 0;
    theta_command = 0;
    command_ << x_command, y_command, theta_command;

    publishCommand();
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

void Controller::publishCommand()
{

  geometry_msgs::Twist vel;
  vel.linear.x = command_(0);
  //vel.linear.x = 0.0;
  vel.linear.y = command_(1);
  //vel.linear.y = 0.0;
 vel.angular.z = command_(2);
  // vel.angular.z = 0.0;

  motor_pub_.publish(vel);

}

void Controller::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{

    gameState_ = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "home");
    ros::NodeHandle nh_;

    Controller controller_node;

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
