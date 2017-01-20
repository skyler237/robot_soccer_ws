#include "AI/skills.h"
#include "Utilities/utilities.h"

#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38

static Vector2d goal_(FIELD_WIDTH/2, 0);

Skills::Skills()
{
  // goal_ << FIELD_WIDTH/2, 0;
}

//=============================================================================
//                            Offensive Skills
//=============================================================================

// Moves to the predicted position of the ball, facing the goal, along a smoothed path
State Skills::ballIntercept(int robotId, State robot, Vector2d ball)
{
  // TODO: Finish this skill
}

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
    destination.theta = theta_d;

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
    destination.theta = theta_d;

    return destination;
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
