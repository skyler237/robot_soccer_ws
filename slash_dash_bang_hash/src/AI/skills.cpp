#include "AI/skills.h"
#include "Utilities/utilities.h"
#include "std_srvs/Trigger.h"
#include <ros/ros.h>
#include <stdio.h>

#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10
#define KICKING_RANGE (0.03 + ROBOT_RADIUS)

static Vector2d goal_(FIELD_WIDTH/2.0, 0);

static double max_xy_vel = 1.0;

// For adaptive radius approach
#define DEFENSE_FOCUS_DEPTH 1.0 // Allows for a more broad radius
#define MAX_RADIUS (DEFENSE_FOCUS_DEPTH + FIELD_HEIGHT/2.0)
#define MIN_RADIUS (DEFENSE_FOCUS_DEPTH + ROBOT_RADIUS)
#define TEAM_AVOID_MARGIN 0.2
#define CONTROL_DELAY 1.0

// For mirror state
#define LEFT 1
#define RIGHT -1

// Ball intercept
#define PATH_AVOIDANCE_MARGIN 0.0
#define EPSILON 0.01

// Get ball
#define POSITION_BEHIND_BALL 0.20
#define DRIBBLE_POSITION_BEHIND_BALL (BALL_RADIUS + 0.01)



Skills::Skills()
{
  // goal_ << FIELD_WIDTH/2, 0;
}

//=============================================================================
//                            General Skills
//=============================================================================

// skill - go to point
//   Travels towards a point. Angle always faces the goal.
State Skills::goToPoint(State robot, Vector2d point)
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
  // ROS_INFO("Robot %d attempted a kick!.", robotId);
  ros::NodeHandle n;

  string robot_number = ((robotId == 1) ? "1" : "2");

  ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("/slash_dash_bang_hash_" + team + "/ally" + robot_number + "/kick");
  std_srvs::Trigger kick_srv;
  client.call(kick_srv);
}

// Get behind the ball, facing a given destination
State Skills::getBall(State robot_state, State ball_state, Vector2d direction_point)
{
  // Normalize vector between ball and destination
  Vector2d ball_vec = stateToVector(ball_state);
  Vector2d n = (direction_point - ball_vec).normalized();

  // compute position 15cm behind ball, but aligned with goal.
  Vector2d position_behind = ball_vec - POSITION_BEHIND_BALL*n;
  Vector2d dribble_position = ball_vec - DRIBBLE_POSITION_BEHIND_BALL*n;

  State destination;
  if(isInFront(robot_state, ball_state, KICKER_WIDTH, POSITION_BEHIND_BALL + 0.05)) {
    destination.x = dribble_position(0);
    destination.y = dribble_position(1);
  }
  else {
    destination.x = position_behind(0);
    destination.y = position_behind(1);
  }
  destination.theta = atan2(direction_point(1) - robot_state.y, direction_point(0) - robot_state.x)*180.0/M_PI;

  return destination;
}

State Skills::hitBallAway(State robot_state, State opp1_state, State opp2_state, State ball_state) {
  // If the opponent is very close, spin the ball away
  const double very_close_to_ball = ROBOT_RADIUS + 2.0*BALL_RADIUS;

  State desired_state = robot_state;

  State close_opponent;
  State other_opponent;
  bool opponent_is_close = false;
  // Check for a close opponent
  if(isInFront(opp1_state, ball_state, ROBOT_RADIUS, very_close_to_ball)) {
    close_opponent = opp1_state;
    other_opponent = opp2_state;
    opponent_is_close = true;
  }
  if(isInFront(opp1_state, ball_state, ROBOT_RADIUS, very_close_to_ball)) {
    close_opponent = opp2_state;
    other_opponent = opp1_state;
    opponent_is_close = true;
  }

  if(opponent_is_close) {
    // Spin the ball away from the side the other oppenent is on
    Vector2d robotToCloseOpponent = stateToVector(close_opponent) - stateToVector(robot_state);
    Vector2d robotToOtherOpponent = stateToVector(other_opponent) - stateToVector(robot_state);

    double other_opponent_perp = vectorProjectedDistance(robotToOtherOpponent, getVecPerpendicularTo(robotToCloseOpponent));

    desired_state = spinRobot(robot_state, -1.0*sgn(other_opponent_perp));
  }
  // If the opponent is not as close, kick the ball into an open spot
  else {
    // Find a spot just past the robot to kick to off the wall
  }

  return desired_state;
}

State Skills::spinRobot(State robot_state, int direction) {
  const double DELTA_THETA = 45.0;

  State rotated_state = robot_state;
  rotated_state.theta = robot_state.theta + DELTA_THETA*sgn(direction);
  return rotated_state;
}


//=============================================================================
//                            Offensive Skills
//=============================================================================



// Returns the y value of the best shot when the ball is in the given position -- includes wall shots
double Skills::findBestShot(State ball_state, State ally_state, State opp1_state, State opp2_state)
{

  State ally_state_left, ally_state_right;
  State opp1_state_left, opp1_state_right;
  State opp2_state_left, opp2_state_right;

  // Keep track of the areas along the wall blocked by each robot
  Zone_t ally_blocked_left, ally_blocked_center, ally_blocked_right;
  Zone_t opp1_blocked_left, opp1_blocked_center, opp1_blocked_right;
  Zone_t opp2_blocked_left, opp2_blocked_center, opp2_blocked_right;

  // Keep track of open spots for each of the goals (actual and mirrored)
  Zone_t goal_open_left, goal_open_center, goal_open_right;
  goal_open_center.max = goal_(1) + GOAL_BOX_WIDTH/2;
  goal_open_center.min = goal_(1) - GOAL_BOX_WIDTH/2;
  goal_open_left.max =   goal_(1) + FIELD_HEIGHT + GOAL_BOX_WIDTH/2;
  goal_open_left.min =   goal_(1) + FIELD_HEIGHT - GOAL_BOX_WIDTH/2;
  goal_open_right.max =  goal_(1) - FIELD_HEIGHT + GOAL_BOX_WIDTH/2;
  goal_open_right.min =  goal_(1) - FIELD_HEIGHT - GOAL_BOX_WIDTH/2;
  Zone_t largest_opening;

  // ROS_INFO("============ Initial openings =============");
  // ROS_INFO("Left opening: min=%f, max=%f", goal_open_left.min, goal_open_left.max);
  // ROS_INFO("Center opening: min=%f, max=%f", goal_open_center.min, goal_open_center.max);
  // ROS_INFO("Right opening: min=%f, max=%f", goal_open_right.min, goal_open_right.max);


  bool considerAlly = false;
  bool considerOpp1 = false;
  bool considerOpp2 = false;

  // Mirror the states of the robots if they are further in x than the ball
  // ROS_INFO("Ball state: x=%f, y=%f", ball_state.x, ball_state.y);
  if(ally_state.x > ball_state.x) {
    considerAlly = true;
    // ROS_INFO("Considering ally in finding best shot.");
  }
  else{
    // ROS_INFO("NOT Considering ally in finding best shot.");
    // ROS_INFO("Ally state: x=%f, y=%f", ally_state.x, ally_state.y);
  }
  if(opp1_state.x > ball_state.x) {
    considerOpp1 = true;
    // ROS_INFO("Considering opp1 in finding best shot.");
  }
  else{
    // ROS_INFO("NOT Considering opp1 in finding best shot.");
    // ROS_INFO("opp1_state: x=%f, y=%f", opp1_state.x, opp1_state.y);
  }
  if(opp2_state.x > ball_state.x) {
    considerOpp2 = true;
    // ROS_INFO("Considering opp2 in finding best shot.");
  }
  else{
    // ROS_INFO("NOT Considering opp2 in finding best shot.");
    // ROS_INFO("opp2_state: x=%f, y=%f", opp2_state.x, opp2_state.y);
  }

  if(considerAlly) {
    ally_state_left = mirrorState(ally_state, LEFT);
    ally_state_right = mirrorState(ally_state, RIGHT);

    ally_blocked_left = findBlockedZone(ball_state, ally_state_left);
    ally_blocked_center = findBlockedZone(ball_state, ally_state);
    ally_blocked_right = findBlockedZone(ball_state, ally_state_right);

    // Update goal open zones
    goal_open_left = updateOpenZone(goal_open_left, ally_blocked_left);
    goal_open_left = updateOpenZone(goal_open_left, ally_blocked_center);

    goal_open_center = updateOpenZone(goal_open_center, ally_blocked_center);

    goal_open_right = updateOpenZone(goal_open_right, ally_blocked_right);
    goal_open_right = updateOpenZone(goal_open_right, ally_blocked_center);
  }
  if(considerOpp1) {
    opp1_state_left = mirrorState(opp1_state, LEFT);
    opp1_state_right = mirrorState(opp1_state, RIGHT);

    opp1_blocked_left = findBlockedZone(ball_state, opp1_state_left);
    opp1_blocked_center = findBlockedZone(ball_state, opp1_state);
    // ROS_INFO("Opp1 blocked center: min=%f, max=%f", opp1_blocked_center.min, opp1_blocked_center.max);
    opp1_blocked_right = findBlockedZone(ball_state, opp1_state_right);

    // Update goal open zones
    goal_open_left = updateOpenZone(goal_open_left, opp1_blocked_left);
    goal_open_left = updateOpenZone(goal_open_left, opp1_blocked_center);

    goal_open_center = updateOpenZone(goal_open_center, opp1_blocked_center);
    // ROS_INFO("Goal center updated (opp1): min=%f, max=%f", goal_open_center.min, goal_open_center.max);

    goal_open_right = updateOpenZone(goal_open_right, opp1_blocked_right);
    goal_open_right = updateOpenZone(goal_open_right, opp1_blocked_center);
  }
  if(considerOpp2) {
    opp2_state_left = mirrorState(opp2_state, LEFT);
    opp2_state_right = mirrorState(opp2_state, RIGHT);

    opp2_blocked_left = findBlockedZone(ball_state, opp2_state_left);
    opp2_blocked_center = findBlockedZone(ball_state, opp2_state);
    opp2_blocked_right = findBlockedZone(ball_state, opp2_state_right);

    // Update goal open zones
    goal_open_left = updateOpenZone(goal_open_left, opp2_blocked_left);
    goal_open_left = updateOpenZone(goal_open_left, opp2_blocked_center);

    goal_open_center = updateOpenZone(goal_open_center, opp2_blocked_center);

    goal_open_right = updateOpenZone(goal_open_right, opp2_blocked_right);
    goal_open_right = updateOpenZone(goal_open_right, opp2_blocked_center);
  }

  // Find the largest open zone
  double largest_right_opening = goal_open_right.max - goal_open_right.min;
  double largest_center_opening = goal_open_center.max - goal_open_center.min;
  double largest_left_opening = goal_open_left.max - goal_open_left.min;

  // ROS_INFO("============ Final openings =============");

  // Compare the three gaps -- gives preference to center shots
  if(largest_left_opening >= largest_center_opening)
  {
    if(largest_left_opening > largest_right_opening)
    {
      largest_opening = goal_open_left;
    }
    else
    {
      largest_opening = goal_open_right;
    }
  }
  else
  {
    if(largest_center_opening > largest_right_opening)
    {
      largest_opening = goal_open_center;
    }
    else
    {
      largest_opening = goal_open_right;
    }
  }

  double midpoint = (largest_opening.max + largest_opening.min)/2.0;

  // double desired_theta = atan(midpoint/(FIELD_WIDTH/2 - ball_state.x));
  // return desired_theta;

  return midpoint;

}


State Skills::makeShot(string team, int robotId, State robot_state, State ball_state, Vector2d ball_destination)
{
  State destination;
  Vector2d robot_pose(robot_state.x, robot_state.y);
  Vector2d ball_pose(ball_state.x, ball_state.y);
  Vector2d robotToGoal = ball_destination - robot_pose;

  Vector2d toBall = ball_pose - robot_pose;
  Vector2d robotForwardVec(cos(robot_state.theta*M_PI/180.0),sin(robot_state.theta*M_PI/180.0));
  double ballForwardDistance = robotForwardVec.dot(toBall)/robotForwardVec.norm();
  // If the robot is close enough to the ball, kick it!
  if (ballForwardDistance < KICKING_RANGE)
  {
    kick(team, robotId);
    destination = robot_state;
  }
  // If not, move towards the ball
  else
  {
    Vector2d desired_pose = ball_pose + 0.05*(toBall.normalized());
    destination = vectorToState(desired_pose);
    destination.theta = atan2(robotToGoal(1),robotToGoal(0))*180.0/M_PI;
  }

  return destination;
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
  // ROS_INFO("Distance comparison: MAX_RADIUS=%f, ballDistance=%f, allyDistance=%f", MAX_RADIUS, ballDistance, allyDistance);
  // Be sure to stay behind the ball and our other team member
  double max_radius = min(ballDistance - ROBOT_RADIUS/2.0, min(MAX_RADIUS, allyDistance - ROBOT_RADIUS - TEAM_AVOID_MARGIN));
  current_radius = saturate(delta_radius + current_radius, MIN_RADIUS, max_radius);
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
  desired_state.theta = angleMod(atan((ball_state.y - goal_(1))/(ball_state.x - goal_(0)))) *180.0 / M_PI; // Always face ball

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

   //Handle the Top/Bottom wall first
   if(prediction(1) > FIELD_HEIGHT/2 || prediction(1) < (-1*(FIELD_HEIGHT / 2)))
   {
       int bounces = 1;

       //get number of times it bounces off walls
       bounces += (int)(((fabs(prediction(1)) - FIELD_HEIGHT/2) / FIELD_HEIGHT));

       //get how far off either end it will be
       double distance_from_wall = fabs(prediction(1)) - FIELD_HEIGHT/2 - FIELD_HEIGHT*(bounces - 1);
       int sign = (prediction(1) > 0) ? 1 : -1; // Positive means top wall first, negative means bottom wall first

       // Map back into field
       if((bounces % 2) == 0)
       {
         //even number of times, off far wall
         prediction(1) = ((-1 * sign) * (FIELD_HEIGHT / 2)) + (sign*distance_from_wall);
       }
       else
       {
         //odd number of times, off close wall
         prediction(1) = ((sign) * (FIELD_HEIGHT / 2)) + ((sign * -1)*distance_from_wall);
       }
   }
   //handle the left/right wall second
   if(prediction(0) > FIELD_WIDTH/2 || prediction(0) < (-1*(FIELD_WIDTH / 2)))
   {
       int bounces = 1;

       //get number of times it bounces off walls
       bounces += (int)(((fabs(prediction(0)) - FIELD_WIDTH/2) / FIELD_WIDTH));

       //get how far off either end it will be
       double distance_from_wall = fabs(prediction(0)) - FIELD_WIDTH/2 - FIELD_WIDTH*(bounces - 1);
       int sign = (prediction(0) > 0) ? 1 : -1;

       if(bounces % 2 == 0)
       {
         //even number of times, off far wall
         prediction(0) = ((-1 * sign) * (FIELD_WIDTH / 2)) + (sign*distance_from_wall);
       }
       else
       {
         //odd number of times, off close wall
         prediction(0) = ((sign) * (FIELD_WIDTH / 2)) + ((sign * -1)*distance_from_wall);
       }
   }
   // Return the estimated position vector
   return prediction;
 }

Vector2d Skills::ballIntercept(State robot_state, State ball_state)
{

  double time1 = 0;
  double time2 = 0;
  double difference = getInterceptDifference(robot_state, ball_state, time2);

  while (difference <= 0)
  {
    time2 += 0.5;
    difference = getInterceptDifference(robot_state, ball_state, time2);
  }

  difference = getInterceptDifference(robot_state, ball_state, (time1 + time2)/2.0);
  int timeout_counter = 100;
  while(fabs(difference) > EPSILON)
  {
    if(difference > 0)
    {
      time2 = (time1 + time2)/2.0;
    }
    else
    {
      time1 = (time1 + time2)/2.0;
    }

    difference = getInterceptDifference(robot_state, ball_state, (time1 + time2)/2.0);

    timeout_counter--;
    if(timeout_counter == 0) {
      ROS_INFO("Ball intercept timed out...");
      break;
    }
  }

  double optimal_time = (time1 + time2)/2.0;

  // ROS_INFO("Optimal time = %f", optimal_time);


  return ballPredict(ball_state, optimal_time);
}

double Skills::getInterceptDifference(State robot_state, State ball_state, double time)
{
  Vector2d robot_pose = stateToVector(robot_state);

  return NOMINAL_VEL*time - ((ballPredict(ball_state, time) - robot_pose).norm() + PATH_AVOIDANCE_MARGIN);
}

State Skills::mirrorState(State robot_state, int direction)
{
  State mirrored_state;
  if (direction == LEFT)
  {
    double y_distance = FIELD_HEIGHT/2 - robot_state.y;

    mirrored_state.y = FIELD_HEIGHT/2 + y_distance;
    mirrored_state.x = robot_state.x;
  }
  else if (direction == RIGHT)
  {
    double y_distance = FIELD_HEIGHT/2 + robot_state.y;

    mirrored_state.y = -FIELD_HEIGHT/2 - y_distance;
    mirrored_state.x = robot_state.x;
  }
  else {
    ROS_INFO("Invalid mirror direction in mirrorState() function");
  }
  mirrored_state.theta = -robot_state.theta;

  return mirrored_state;
}

Zone_t Skills::findBlockedZone(State ball_state, State blocker_state)
{
  Zone_t blocked_zone;
  Vector2d ball_pose(ball_state.x , ball_state.y);
  Vector2d blocker_pose(blocker_state.x, blocker_state.y);
  // Vector2d robot_pose(robot_state.x, robot_state.y);

  double ball_dist_from_wall = FIELD_WIDTH/2 - ball_pose(0);

  Vector2d ballToBlocker = blocker_pose - ball_pose;

  double ball_evade_radius = ROBOT_RADIUS + BALL_RADIUS*2;
  Vector2d blocker_left_edge, blocker_right_edge;

  double theta_edge = atan(ball_evade_radius/ballToBlocker.norm());
  double theta_blocker = atan(ballToBlocker(1)/ballToBlocker(0));
  double theta_evade_left = theta_blocker + theta_edge;
  double theta_evade_right = theta_blocker - theta_edge;

  double dy_left = ball_dist_from_wall*tan(theta_evade_left);
  double dy_right = ball_dist_from_wall*tan(theta_evade_right);

  blocked_zone.max = ball_pose(1) + dy_left;
  blocked_zone.min = ball_pose(1) + dy_right;

  return blocked_zone;
}

Zone_t Skills::updateOpenZone(Zone_t open_zone, Zone_t blocked_zone)
{
  // For comments let "O" represent open zone boundary and "B" reresent blocked zone boundary
  Zone_t updated_open_zone = open_zone;
  if(open_zone.min == open_zone.max) // There is no open area left, just return
  {
    return updated_open_zone;
  }

  if(blocked_zone.max > open_zone.min && blocked_zone.max < open_zone.max) // Check if the blocked max is in the open region
  {
    // O----B--0---B = blocked zone straddles right edge
    if(blocked_zone.min < open_zone.min) // Does it straddle?
    {
      updated_open_zone.min = blocked_zone.max;
    }
    else
    {
      // O----B---B-O = blocked zone inside, but bigger opening on the left
      if(open_zone.max - blocked_zone.max > blocked_zone.min - open_zone.min)
      {
        updated_open_zone.min = blocked_zone.max;
      }
      // O-B---B-----O = = blocked zone inside, but bigger opening on the right
      else
      {
        updated_open_zone.max = blocked_zone.min;
      }
    }
  }
  else if(blocked_zone.min > open_zone.min && blocked_zone.min < open_zone.max) // Check if the blocked min is in the open region
  {
    // B--O---B--O = blocked zone straddles the left edge
    updated_open_zone.max = blocked_zone.min;
  }
  else if(blocked_zone.min < open_zone.min && blocked_zone.max > open_zone.max) // Check if the blocked area completely covers the open area
  {
    // B--O-----O--B = blocked zone completely covers the open zone
    updated_open_zone.max = updated_open_zone.min;
  }

  return updated_open_zone;
}

// This is just for testing
// 1 = back left, 2 = goal left, 3 = goal right, 4 = back right
State Skills::hideInCorner(int corner_number)
{
    State desired_state;
    switch (corner_number) {
      case 1: // Back left (own side)
        desired_state.x = -FIELD_WIDTH/2;
        desired_state.y = FIELD_HEIGHT/2;
        break;

      case 2: // Goal left (own side)
        desired_state.x = -FIELD_WIDTH/2 - ROBOT_RADIUS;
        desired_state.y = GOAL_BOX_WIDTH/2 - ROBOT_RADIUS;
        break;

      case 3: // Goal right (own side)
        desired_state.x = -FIELD_WIDTH/2 - ROBOT_RADIUS;
        desired_state.y = -(GOAL_BOX_WIDTH/2 - ROBOT_RADIUS);
        break;

      case 4: // Back right (own side)
        desired_state.x = -FIELD_WIDTH/2;
        desired_state.y = -FIELD_HEIGHT/2;
        break;
    }

    return desired_state;
}


//=============================================================================
//                            Skills Competition Functions
//=============================================================================

// Spins clockwise for 3 seconds then counter-clockwise for 3 seconds (repeatedly)
State Skills::spinInPlace(State robot_state)
{
  typedef enum {
    RESET,
    SPIN_CW,
    SPIN_CCW
  } state_t;
  const double delta_theta = 90.0; // Degrees
  const double state_switch_max_value = 3.0; // seconds
  const double timeout_max_value = 3.0; // seconds

  static state_t skill_state = SPIN_CW;

  double now = ros::Time::now().toSec();
  static double prev_time = 0;
  double dt = now - prev_time;
  prev_time = now;

  static double state_switch_timer = 0;

  State destination;
  destination = robot_state;
  double desired_theta = 0;

  switch (skill_state) {
    case RESET:
      skill_state = SPIN_CW;
      state_switch_timer = 0.0;
      break;

    case SPIN_CW:
      // State transition
      if(state_switch_timer > state_switch_max_value) {
        skill_state = SPIN_CCW;
        state_switch_timer = 0.0;
      }
      else if(dt > timeout_max_value) {
        skill_state = RESET;
      }
      else {
        skill_state = SPIN_CW;
        state_switch_timer += dt;
      }

      // State actions
      desired_theta = fmod(robot_state.theta + delta_theta, 360.0);
      destination.theta = desired_theta;
      break;

    case SPIN_CCW:
      // State transition
      if(state_switch_timer > state_switch_max_value) {
        skill_state = SPIN_CW;
        state_switch_timer = 0.0;
      }
      else if(dt > timeout_max_value) {
        skill_state = RESET;
      }
      else {
        skill_state = SPIN_CCW;
        state_switch_timer += dt;
      }

      // State actions
      desired_theta = fmod(robot_state.theta - delta_theta + 360.0, 360.0);
      destination.theta = desired_theta;
      break;

    default:
      ROS_INFO("Error: invalid state in spinInPlace() skill function");
  }

  return destination;
}

State Skills::goToCenterFacingGoal()
{
  State destination;
  destination.x = 0.0;
  destination.y = 0.0;
  destination.theta = 0.0;

  return destination;
}

// ------- GOAL --------
//  1  4         4  1  |
//  2  3         3  2  |
//         OR          |
//  2  3         3  2  |
//  1  4         4  1  |
//-----------------------
State Skills::moveInBoxFacingGoal(State robot_state)
{
  State destination;
  typedef enum {
    RESET,
    GO_TO_CORNER1,
    GO_TO_CORNER2,
    GO_TO_CORNER3,
    GO_TO_CORNER4
  } state_t;
  const double timeout_max_value = 2.0; // seconds
  const double waypoint_epsilon = 0.1;

  static state_t skill_state = RESET;

  double now = ros::Time::now().toSec();
  static double prev_time = 0;
  double dt = now - prev_time;
  prev_time = now;

  Vector2d robot_pose = stateToVector(robot_state);

  const Vector2d dx(0.3, 0.0);
  const Vector2d dy(0.0, 0.3);

  static Vector2d corner1 = robot_pose;
  static Vector2d corner2 = corner1 - sgn(robot_state.x)*dx;
  static Vector2d corner3 = corner2 - sgn(robot_state.y)*dy;
  static Vector2d corner4 = corner3 + sgn(robot_state.x)*dx;

  switch (skill_state) {
    case RESET:
      skill_state = GO_TO_CORNER2;

      // Reassign the corners
      corner1 = stateToVector(robot_state);
      corner2 = corner1 - sgn(robot_state.x)*dx;
      corner3 = corner2 - sgn(robot_state.y)*dy;
      corner4 = corner3 + sgn(robot_state.x)*dx;
      break;

    case GO_TO_CORNER1:
      // State transition
      if((corner1 - robot_pose).norm() < waypoint_epsilon) {
        skill_state = GO_TO_CORNER2;
      }
      else if(dt > timeout_max_value) {
        skill_state = RESET;
      }
      else {
        skill_state = GO_TO_CORNER1;
      }

      // State actions
      destination = vectorToState(corner1);
      break;

    case GO_TO_CORNER2:
      // State transition
      if((corner2 - robot_pose).norm() < waypoint_epsilon) {
        skill_state = GO_TO_CORNER3;
      }
      else if(dt > timeout_max_value) {
        skill_state = RESET;
      }
      else {
        skill_state = GO_TO_CORNER2;
      }

      // State actions
      destination = vectorToState(corner2);
      break;

    case GO_TO_CORNER3:
      // State transition
      if((corner3 - robot_pose).norm() < waypoint_epsilon) {
        skill_state = GO_TO_CORNER4;
      }
      else if(dt > timeout_max_value) {
        skill_state = RESET;
      }
      else {
        skill_state = GO_TO_CORNER3;
      }

      // State actions
      destination = vectorToState(corner3);
      break;

    case GO_TO_CORNER4:
      // State transition
      if((corner4 - robot_pose).norm() < waypoint_epsilon) {
        skill_state = GO_TO_CORNER1;
      }
      else if(dt > timeout_max_value) {
        skill_state = RESET;
      }
      else {
        skill_state = GO_TO_CORNER4;
      }

      // State actions
      destination = vectorToState(corner4);
      break;

    default:
      ROS_INFO("Error: invalid state in spinInPlace() skill function");
  }

  destination.theta = 0.0;
  return destination;
}
