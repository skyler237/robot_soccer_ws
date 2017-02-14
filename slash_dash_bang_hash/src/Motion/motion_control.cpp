#include "Motion/motion_control.h"
#include "Utilities/utilities.h"


MotionControl::MotionControl() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");

  // Define kinematics matrix
  M_ << SX1, SY1, (SY1*RX1 - SX1*RY1),
        SX2, SY2, (SY2*RX2 - SX2*RY2),
        SX3, SY3, (SY3*RX3 - SX3*RY3);
  M_ = 1/RHO*M_;

  velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("vel_command", 1, &MotionControl::velocityCallback, this);
  robot_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("robot_state", 1, &MotionControl::robotStateCallback, this);

  motor_speed_pub_ = nh_.advertise<slash_dash_bang_hash::MotorSpeeds>("motor_speeds", 5);
  //ROS_INFO("Init");

}

void MotionControl::velocityCallback(const TwistConstPtr &msg)
{
  desired_velocity_ = *msg;
  computeMotorSpeeds();
}

void MotionControl::robotStateCallback(const StateConstPtr &msg)
{
  robot_state_ = *msg;
}

void MotionControl::computeMotorSpeeds() {
  double theta = robot_state_.theta;
  double ct = cos(theta);
  double st = sin(theta);

  Matrix3d R;
  R    <<     ct,  st, 0.0,
             -st,  ct, 0.0,
             0.0, 0.0, 1.0;

  Vector3d velocities(desired_velocity_.linear.x, desired_velocity_.linear.y, desired_velocity_.angular.z);

  Vector3d wheel_speeds = M_*R*velocities;

  motor_speeds_.motor1_speed = wheel_speeds(0);
  motor_speeds_.motor2_speed = wheel_speeds(1);
  motor_speeds_.motor3_speed = wheel_speeds(2);

  publishSpeeds();
}

void MotionControl::publishSpeeds()
{

  motor_speed_pub_.publish(motor_speeds_);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "home");
    ros::NodeHandle nh_;

    MotionControl motion_node;

    ros::spin();
}
