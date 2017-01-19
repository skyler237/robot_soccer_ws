#include "utilities.h"


static int Utilities::sgn(double val)
{
    return (0 < val) - (val < 0);
}

// utility - saturate velocity
//  Saturate the commanded velocity .
static Vector3d Utilities::saturateVelocity(Vector3d v)
{
    if(fabs(v(0)) > ROBOT_MAX_VXY)
        v(0) = sgn(v(0)) * ROBOT_MAX_VXY;
    if(fabs(v(1)) > ROBOT_MAX_VXY)
        v(1) = sgn(v(1)) * ROBOT_MAX_VXY;
    if(fabs(v(2)) > ROBOT_MAX_OMEGA)
        v(2) = sgn(v(2)) * ROBOT_MAX_OMEGA;
    return v;
}

static double Utilities::angleMod(double angle)
{
    while(angle < 0)
        angle += 2*M_PI;
    return fmod(angle + M_PI, (2*M_PI)) - M_PI;
}

static State Utilities::poseToState(Pose2D robot)
{
    State state;
    // Flip coordinates if team is away or if we've swapped sides
    if((team == "away") ^ gameState.second_half)
    {
        state.x = -robot.x;
        state.y = -robot.y;
        state.theta = angleMod(robot.theta + M_PI);
    }

    return state;
}

static State Utilities::vectorToState(Vector2d vec)
{
  State state;
  state.x = vec(0);
  state.y = vec(1);

  return state;
}

static Vector2d Utilities::stateToVector(State robot)
{
  Vector2d vec(robot.x, robot.y);
  return vec;
}
