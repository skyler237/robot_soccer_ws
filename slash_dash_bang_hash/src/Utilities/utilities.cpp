#include "Utilities/utilities.h"


int sgn(double val)
{
    return (0 < val) - (val < 0);
}
// utility - saturate velocity
//  Saturate the commanded velocity .
Vector3d saturateVelocity(Vector3d v)
{
    if(fabs(v(0)) > ROBOT_MAX_VXY)
        v(0) = sgn(v(0)) * ROBOT_MAX_VXY;
    if(fabs(v(1)) > ROBOT_MAX_VXY)
        v(1) = sgn(v(1)) * ROBOT_MAX_VXY;
    if(fabs(v(2)) > ROBOT_MAX_OMEGA)
        v(2) = sgn(v(2)) * ROBOT_MAX_OMEGA;
    return v;
}

double saturate(double x, double min, double max)
{
  if (x < min) {
    return min;
  }
  else if ( x > max ) {
    return max;
  }
  else {
    return x;
  }

}

double angleMod(double angle)
{
    while(angle < 0)
        angle += 2*M_PI;
    return fmod(angle + M_PI, (2*M_PI)) - M_PI;
}

// slash_dash_bang_hash::State poseToState(geometry_msgs::Pose2D robot)
// {
//     State state;
//     // Flip coordinates if team is away or if we've swapped sides
//     if((team == "away") ^ gameState.second_half)
//     {
//         state.x = -robot.x;
//         state.y = -robot.y;
//         state.theta = angleMod(robot.theta + M_PI);
//     }
//
//     return state;
// }

slash_dash_bang_hash::State vectorToState(Vector3d vec)
{
  slash_dash_bang_hash::State state;
  state.x = vec(0);
  state.y = vec(1);
  state.theta = vec(2);

  return state;
}

slash_dash_bang_hash::State vectorToState(Vector2d vec)
{
  slash_dash_bang_hash::State state;
  state.x = vec(0);
  state.y = vec(1);

  return state;
}

 Vector2d stateToVector(slash_dash_bang_hash::State robot)
{
  Vector2d vec(robot.x, robot.y);
  return vec;
}

void printVector(Vector2d vector, string name)
{
  ROS_INFO("%s: x=%f, y=%f", name.c_str(), vector(0), vector(1));
}

Vector2d getVecPerpendicularTo(Vector2d vec)
{
  Vector2d perp(-1.0*vec(1), vec(0));
  return perp;
}

double vectorProjectedDistance(Vector2d vec, Vector2d reference)
{
  double proj_dist = vec.dot(reference)/reference.norm();
  return proj_dist;
}

bool isInFront(State robot, State object, double box_radius, double box_length)
{
  Vector2d robot_pose = stateToVector(robot);
  Vector2d object_pose = stateToVector(object);
  Vector2d robotForwardVec(cos(robot.theta*M_PI/180.0),sin(robot.theta*M_PI/180.0));
  Vector2d forwardPerp = getVecPerpendicularTo(robotForwardVec);

  Vector2d toObject = object_pose - robot_pose;

  double object_forward_dist = vectorProjectedDistance(toObject, robotForwardVec);
  double object_perp_dist = vectorProjectedDistance(toObject, forwardPerp);

  return (object_forward_dist <= box_length) && (fabs(object_perp_dist) <= box_radius);
}

bool isObjectBetween(State object, State robot1, State robot2)
{
  Vector2d robot1_pose = stateToVector(robot1);
  Vector2d robot2_pose = stateToVector(robot2);
  Vector2d object_pose = stateToVector(object);
  Vector2d betweenRobots(robot2_pose - robot1_pose);
  Vector2d perp = getVecPerpendicularTo(betweenRobots);

  Vector2d toObject = object_pose - robot1_pose;

  double object_forward_dist = vectorProjectedDistance(toObject, betweenRobots);
  double object_perp_dist = vectorProjectedDistance(toObject, perp);

  return (object_forward_dist <= betweenRobots.norm() && object_forward_dist > 0.0) && (fabs(object_perp_dist) <= ROBOT_RADIUS);
}

bool ballIsInPossessionOf(State robot, State ball)
{
  return isInFront(robot, ball, KICKER_WIDTH/2, POSSESSION_RANGE);
}
