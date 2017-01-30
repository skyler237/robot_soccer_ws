#include "AI/skills.h"
#include "Utilities/utilities.h"
#include "std_srvs/Trigger.h"
#include <ros/ros.h>
#include <stdio.h>

#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10

static Vector2d goal_(FIELD_WIDTH/2.0, 0);

static double max_xy_vel = 1.0;

// For adaptive radius approach
#define DEFENSE_FOCUS_DEPTH 1.0 // Allows for a more broad radius
#define MAX_RADIUS (DEFENSE_FOCUS_DEPTH + FIELD_HEIGHT/2.0)
#define MIN_RADIUS (DEFENSE_FOCUS_DEPTH + ROBOT_RADIUS)
#define TEAM_AVOID_MARGIN 0.05
#define CONTROL_DELAY 1.0

Skills::Skills()
{
  // goal_ << FIELD_WIDTH/2, 0;
}

//=============================================================================
//                            General Skills
//=============================================================================

// skill - go to point
//   Travels towards a point. Angle always faces the goal.
State Skills::goToPoint(int robotId, State robot, Vector2d point)
{
    // control angle to face the goal
    Vector2d dirGoal = goal_ - stateToVector(robot);
    double theta_d = atan2(dirGoal(1), dirGoal(0));

    State destination;
    destination.x = point(0);
    destination.y = point(1);
    destination.theta = angleMod(theta_d)*180.0/M_PI;

    //ROS_INFO("Go to point desired theta = %f", destination.theta);

    return destination;
}

void Skills::kick(string team, int robotId)
{
  ROS_INFO("Robot %d attempted a kick!.", robotId);
  ros::NodeHandle n;

  string robot_number = ((robotId == 1) ? "1" : "2");

  ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("/slash_dash_bang_hash_" + team + "/ally" + robot_number + "/kick");
  std_srvs::Trigger kick_srv;
  client.call(kick_srv);
}

// Get behind the ball, facing a given destination -- avoids hitting the ball backward
State Skills::getBall(State robot_state, State ball_state, Vector2d direction_point)
{
  // Normalize vector between ball and destination
  Vector2d ball_vec = stateToVector(ball_state);
  Vector2d n = (direction_point - ball_vec).normalized();

  // compute position 15cm behind ball, but aligned with goal.
  Vector2d final_position = ball_vec - 0.15*n;

  State destination;
  destination.x = final_position(0);
  destination.y = final_position(1);

  return destination;
}

//=============================================================================
//                            Offensive Skills
//=============================================================================

// Moves to the predicted position of the ball, facing the goal, along a smoothed path
State Skills::ballIntercept(int robotId, State robot, Vector2d ball)
{
  // TODO: Finish this skill

  // Vector2d ballToGoal;
  // ballToGoal.x = ball_state_.x - goal_(0);
  // ballToGoal.y = ball_state_.y - goal_(0);
}



//=============================================================================
//                            Defensive Skills
//=============================================================================

// skill - follow ball on line
//   Follows the y-position of the ball, while maintaining x-position at x_pos.
//   Angle always faces the goal.
State Skills::followBallOnLine(int robotId, State robot, State ball, double x_pos)
{
    // control angle to face the goal
    Vector2d dirGoal = goal_ - stateToVector(robot);
    double theta_d = atan2(dirGoal(1), dirGoal(0));

    State destination;
    destination.x = x_pos;
    destination.y = ball.y;
    destination.theta = angleMod(theta_d*180.0/M_PI);;


    return destination;
}

State Skills::adaptiveRadiusGoalDefend(State robot_state, State ally_state, State ball_state) {
  Vector2d defense_origin(-1.0*(goal_(0) + DEFENSE_FOCUS_DEPTH), goal_(1));

  double now = ros::Time::now().toSec();
  static double prev_time = 0;
  static double current_radius = fabs(robot_state.x - defense_origin(0));
  double dt = now - prev_time;
  prev_time = now;

  // Calculate theta (target approach angle (relative to line of sight vector))
  Vector2d robot_pos(robot_state.x, robot_state.y);
  Vector2d ball_pos(ball_state.x, ball_state.y);

  Vector2d v_ball(ball_state.xdot, ball_state.ydot);
  Vector2d v_robot(robot_state.xdot, robot_state.ydot);
  Vector2d d = (ball_pos - defense_origin) - current_radius*(ball_pos - defense_origin).normalized(); // distance from the ball to the current radius
  double v_ball_norm = v_ball.norm(); // Magnitude of ball velocity
  double d_norm = d.norm();
  double theta_robot = atan(ROBOT_RADIUS/3 / d_norm); // Angle from target to edge of "defense area" of robot
  double theta;
  if(v_ball_norm != 0 && d_norm != 0) { // Protect against divide by zero
     // Only count theta deviations greater than the angle to the edge of the "defense area"
     // Any angle within the range of the robot body is counted as zero.
    theta = saturate(fabs(d.dot(v_ball)/(v_ball_norm*d_norm)) - theta_robot, 0.0, M_PI);
  }
  else {
    theta = 0.0;
  }

  // Calculate phi (robot divergent angle) using bisection search
  double phi1 = -1.0*M_PI/2.0;
  double phi2 = M_PI/2.0;
  double phi = (phi1 + phi2)/2.0;
  // double max_xy_vel = v_robot.norm();
  double t_robot = (d_norm*(sin(theta)))/(max_xy_vel*(cos(theta + phi))) + CONTROL_DELAY;
  double t_ball = (d_norm*(cos(phi)) / (v_ball_norm*(cos(theta+phi))));
  double diff = t_ball - t_robot;


  double epsilon = 0.01;
  int timeout = 5000;
  while(fabs(diff) > epsilon && theta != 0) { // If theta = 0, skip this loop
    phi = (phi1 + phi2)/2.0;
    t_robot = (d_norm*(sin(theta)))/(max_xy_vel*(cos(theta + phi))) + CONTROL_DELAY;
    t_ball = (d_norm*(cos(phi))/(v_ball_norm*(cos(theta+phi))));
    diff = t_ball - t_robot;
    if(diff < 0) {
      phi1 = phi;
    }
    else {
      phi2 = phi;
    }

    timeout--;
    if(timeout == 0) {
      // phi = 0.0;
      break;
    }
  }

  double delta_radius;
  if(theta == 0) {
    // Don't change radius if theta is zero
    delta_radius = 0;
  }
  else {
    delta_radius = -1.0*max_xy_vel*dt*sin(phi);
  }

  double ballDistance = (ball_pos - defense_origin).norm();
  Vector2d ally_pos(ally_state.x, ally_state.y);
  double allyDistance = (ally_pos - defense_origin).norm();
  ROS_INFO("Distance comparison: MAX_RADIUS=%f, ballDistance=%f, allyDistance=%f", MAX_RADIUS, ballDistance, allyDistance);
  // Be sure to stay behind the ball and our other team member
  current_radius = saturate(delta_radius + current_radius, MIN_RADIUS, min(ballDistance - ROBOT_RADIUS/2.0, min(MAX_RADIUS, allyDistance - ROBOT_RADIUS - TEAM_AVOID_MARGIN)));
  // if(debug_print & ADAPTIVE_RADIUS) {
    // ROS_INFO("PHI COMPUTATIONS >>>>>>>>>>>>>>>>>>>>>>");
    // ROS_INFO("d = %f", d_norm);
    // ROS_INFO("max_xy_vel = %f", max_xy_vel);
    // ROS_INFO("v_ball_norm = %f", v_ball_norm);
    // ROS_INFO("Theta_deg = %f", theta/M_PI*180.0);
    // ROS_INFO("Phi_deg = %f", phi/M_PI*180.0);
    // ROS_INFO("Delta_radius = %f", delta_radius);
    // ROS_INFO("Current radius = %f", current_radius);
  // }


  // Protection Radius defense position
  Vector2d desired_pose = (ball_pos - defense_origin).normalized()*current_radius;

  // ROS_INFO("Desired pose: x=%f, y=%f", desired_pose(0), desired_pose(1));

  State desired_state;
  desired_state.x = saturate(defense_origin(0) + desired_pose(0), -(FIELD_WIDTH/2 - ROBOT_RADIUS/2), (FIELD_WIDTH/2 - ROBOT_RADIUS/2)); // Never try to go past walls...
  desired_state.y = saturate(defense_origin(1) + desired_pose(1), -(FIELD_HEIGHT/2 - ROBOT_RADIUS/2), (FIELD_HEIGHT/2 - ROBOT_RADIUS/2));

  // ROS_INFO("Desired state: x=%f, y=%f", desired_state.x, desired_state.y);
  //desired_state.theta = angleMod(atan((ball_state.y - defense_origin(1))/(ball_state.x - defense_origin(0)))*180.0/M_PI); // Always face ball from center of goal
  desired_state.theta = angleMod(atan((ball_state.y - defense_origin(1))/(ball_state.x - defense_origin(0)))) *180.0 / M_PI; // Always face ball from center of goal

  //ROS_INFO("Adaptive Radius desired theta = %f", desired_state.theta);

  return desired_state;
}

//=============================================================================
//                            Helper Functions
//=============================================================================

/**
 * Returns the predicted position of the target a given time away from the present
 * @param time - the time into the future we want to predict the target position
 * @return Vector2d - predicted position
 */
Vector2d Skills::ballPredict(State ball, double time) {

  // Extrapolate the specified amount of time into the future
  Vector2d prediction(ball.x, ball.y); // Initialize with current position
  Vector2d velocity(ball.xdot, ball.ydot);

  // Extrapolate the ball position
  prediction += time*velocity;

  // TODO: Handle the walls
  //Handle the Top/Bottom wall first
  if(prediction(1) > FIELD_HEIGHT / 2|| prediction(1) < (-1*(FIELD_HEIGHT / 2)))
  {
      //get number of times it bounces off walls
      int bounces = abs((int)(prediction(1) / FIELD_HEIGHT));
      //get how far off either end it will be
      double yAbs = (abs(prediction(1) / FIELD_HEIGHT) - (bounces - 1)) * FIELD_HEIGHT;
      int sign = (prediction(1) > 0) ? 1 : -1;
      if((bounces % 2) == 0)
      {
        //even number of times, off far wall
        prediction(1) = ((-1 * sign) * (FIELD_HEIGHT / 2)) + (sign*yAbs);
      }
      else
      {
        //odd number of times, off close wall
        prediction(1) = ((sign) * (FIELD_HEIGHT / 2)) + ((sign * -1)*yAbs);
      }
  }
  //handle the left/right wall second
  if(prediction(0) > FIELD_WIDTH / 2|| prediction(0) < (-1*(FIELD_WIDTH / 2)))
  {
      //get number of times it bounces off walls
      int bounces = abs((int)(prediction(0) / FIELD_WIDTH));
      //get how far off either end it will be
      double yAbs = (abs(prediction(0) / FIELD_HEIGHT) - (bounces - 1)) * FIELD_WIDTH;
      int sign = (prediction(0) > 0) ? 1 : -1;
      if(bounces % 2 == 0)
      {
        //even number of times, off far wall
        prediction(0) = ((-1 * sign) * (FIELD_WIDTH / 2)) + (sign*yAbs);
      }
      else
      {
        //odd number of times, off close wall
        prediction(0) = ((sign) * (FIELD_WIDTH / 2)) + ((sign * -1)*yAbs);
      }
  }
  // Return the estimated position vector
  return prediction;
}
