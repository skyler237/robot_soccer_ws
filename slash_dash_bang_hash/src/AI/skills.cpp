#include "slash_dash_bang_hash/skills.h"

#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38

//=============================================================================
//                            Offensive Skills
//=============================================================================

// Moves to the predicted position of the ball, facing the goal, along a smoothed path
static State Skills::ballIntercept(int robotId, State robot, Vector2d ball)
{
  // TODO: Finish this skill
}

// skill - go to point
//   Travels towards a point. Angle always faces the goal.
static State Skills::goToPoint(int robotId, State robot, Vector2d point)
{
    Vector2d dirPoint = point - Utilities::stateToVector(robot);
    Vector2d vxy = dirPoint * CONTROL_K_XY;

    // control angle to face the goal
    Vector2d dirGoal = goal - Utilities::stateToVector(robot);
    double theta_d = atan2(dirGoal(1), dirGoal(0));
    double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d);

    // Output velocities to motors
    Vector2d v;
    v << vxy, omega;
    v = Utilities::saturateVelocity(v);
    return Utilities::vectorToState(v);
}

//=============================================================================
//                            Defensive Skills
//=============================================================================

// skill - follow ball on line
//   Follows the y-position of the ball, while maintaining x-position at x_pos.
//   Angle always faces the goal.
static State Skills::followBallOnLine(int robotId, State robot, Vector2d ball, double x_pos)
{
    // control x position to stay on current line
    double vx = CONTROL_K_XY * (x_pos - robot.x);

    // control y position to match the ball's y-position
    double vy = CONTROL_K_XY * (ball_.y - robot.y);

    // control angle to face the goal
    Vector2d dirGoal = goal - Utilities::stateToVector(robot);
    double theta_d = atan2(dirGoal(1), dirGoal(0));
    double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d);

    // Output velocities to motors
    Vector2d v;
    v << vx, vy, omega;
    v = Utilities::saturateVelocity(v);
    return Utilities::vectorToState(v);
}

//=============================================================================
//                            Helper Functions
//=============================================================================

/**
 * Returns the predicted position of the target a given time away from the present
 * @param time - the time into the future we want to predict the target position
 * @return Vector2d - predicted position
 */
Eigen::Vector2d Skills::ballPredict(State ball, double time) {

  // Extrapolate the specified amount of time into the future
  Eigen::Vector2d prediction(ball.x, ball.y); // Initialize with current position
  Eigen::Vector2d velocity(ball.xdot, ball.ydot);

  // Extrapolate the ball position
  prediction += time*velocity;

  // TODO: Handle the walls


  // Return the estimated position vector
  return prediction;
}
