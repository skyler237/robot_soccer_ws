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
