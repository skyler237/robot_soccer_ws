#include "AI/ai.h"
#include "Utilities/utilities.h"
#include "AI/skills.h"

#define AI_TIME_STEP 0.00
#define POSITION_BEHIND_BALL 0.20

AI::AI() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");
  priv_nh.param<string>("team", team_, "home");
  priv_nh.param<int>("robot_number", robot_number_, 1);


  ally1_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally1_state", 1, boost::bind(&AI::stateCallback, this, _1, "ally1"));
  ally2_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally2_state", 1, boost::bind(&AI::stateCallback, this, _1, "ally2"));
  opp1_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("opp1_state", 1, boost::bind(&AI::stateCallback, this, _1, "opponent1"));
  opp2_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("opp2_state", 1, boost::bind(&AI::stateCallback, this, _1, "opponent2"));
  ball_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ball_state", 1, boost::bind(&AI::stateCallback, this, _1, "ball"));
  game_state_sub_ = nh_.subscribe<soccerref::GameState>("/game_state", 1, &AI::gameStateCallback, this);

  ally1_destination_pub_ = nh_.advertise<slash_dash_bang_hash::State>("ally1_destination", 5);
  ally2_destination_pub_ = nh_.advertise<slash_dash_bang_hash::State>("ally2_destination", 5);

  // This is sort of ad-hoc (would be much better to be a parameter) but it works for now
  ally1_startingPos_(0) = -0.5;
  ally1_startingPos_(1) = -0.0;

  ally2_startingPos_(0) = -1.0;
  ally2_startingPos_(1) = -0.0;

  goal_ << FIELD_WIDTH/2, 0;

}

void AI::param_init()
{

}

void AI::computeDestination() {
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  if (dt > AI_TIME_STEP)
  {
    if (gameState_.play)
    {
        /*********************************************************************/
        // Choose strategies and update ally1/ally2_destination_ variables

        // robot #1 positions itself behind ball and rushes the goal.
        // ally1_destination_ = play_findBestShot(1, ally1_state_, ball_state_);
        // ally1_destination_ = play_rushGoal(ally1_state_, ball_state_);
        ally1_destination_ = play_standardOffense();
        // ally1_destination_ = play_skillsTournament(ally1_state_);

        // checkForKick(1);
        //ROS_INFO("Ally1_destination: x=%f, y=%f", ally1_destination_.x, ally1_destination_.y);



        // robot #2 defend the goal
        // ally2_destination_ = play_basicDefense(ally2_state_, ball_state_);
        ally2_destination_ = Skills::hideInCorner(1); // Hide in back left corner -- for debugging
        checkForKick(2);

        ///////////////////////////////////////////////////////
        ////////////////////DEBUG//////////////////////////////
        //Vector2d dirGoal;
        //dirGoal << 1, 1;
        ///////////////////////////////////////////////////////
        //go to the center of the field, as a test to see if our controll is correct
        //ally2_destination_ = Skills::goToPoint(ally2_state_, dirGoal);
        //////////////////////////////////////////////////////////




        //ROS_INFO("Ally2_destination: x=%f, y=%f", ally2_destination_.x, ally2_destination_.y);
          //ROS_INFO("positionx: %d   positiony: %d", x_pos, ball.y);
        /*********************************************************************/

        publishDestinations();
    }
    else if (gameState_.reset_field)
    {
        ally1_destination_ = Skills::goToPoint(ally1_state_, ally1_startingPos_);
        ally2_destination_ = Skills::goToPoint(ally2_state_, ally2_startingPos_);

        publishDestinations();
    }
  }

}

void AI::checkForKick(int robotId)
{
  State robot_state;
  switch (robotId) {
    case 1:
      robot_state = ally1_state_;
      break;
    case 2:
      robot_state = ally2_state_;
      break;
  }

  Vector2d robot_pose = stateToVector(robot_state);
  Vector2d ball_pose = stateToVector(ball_state_);

  // If we are close enough, attempt a kick
  // TODO: check if the ball is in front of us or not...
  if(isInFront(robot_state, ball_state_, KICKER_WIDTH/2, KICKING_RANGE))
  {
      // TODO: check for minimum re-kick time?
      Skills::kick(team_, robotId);
  }
}

void AI::publishDestinations()
{
  ally1_destination_pub_.publish(ally1_destination_);
  ally2_destination_pub_.publish(ally2_destination_);
}


// play - rush goal
//   - go to position behind ball
//   - if ball is between robot and goal, go to goal
// NOTE:  This is a play because it is built on skills, and not control
// commands.  Skills are built on control commands.  A strategy would employ
// plays at a lower level.  For example, switching between offense and
// defense would be a strategy.
State AI::play_rushGoal(State robot, State ball)
{
    // normal vector from ball to goal
    // Vector2d ball_vec = stateToVector(ball);
    Vector2d ball_vec = Skills::ballIntercept(robot, ball);
    ROS_INFO("ball_vel: xdot=%f, ydot=%f", ball.xdot, ball.ydot);
    printVector(ball_vec, "ball_vec");
    Vector2d n = (goal_ - ball_vec).normalized();

    // compute position 10cm behind ball, but aligned with goal.
    Vector2d position = ball_vec - POSITION_BEHIND_BALL*n;

    // if((position - stateToVector(robot)).norm() < 0.21)
    if(isInFront(robot, ball, KICKER_WIDTH/2, POSITION_BEHIND_BALL + 0.05))
        return Skills::goToPoint(robot, goal_);
    else
        return Skills::goToPoint(robot, position);
}

State AI::play_findBestShot(int robotId, State robot, State ball)
{
  // normal vector from ball to goal
  Vector2d ball_vec = stateToVector(ball);
  double open_zone_midpoint = Skills::findBestShot(ball, ally1_state_, opp1_state_, opp2_state_);
  ROS_INFO("Shot midpoint: y=%f", open_zone_midpoint);
  Vector2d shot_destination(FIELD_WIDTH/2.0, open_zone_midpoint);

  // TODO: Does not dribble the ball...
  if((ball_vec - stateToVector(robot)).norm() < 0.21)
      return Skills::makeShot(team_, robot_number_, robot, ball, shot_destination);
  else
      return Skills::getBall(robot, ball, shot_destination);
}

State AI::play_basicDefense(State robot, State ball)
{

  // normal vector from ball to goal
  Vector2d ball_vec = stateToVector(ball);
  Vector2d robot_vec = stateToVector(robot);

  //if the ball is close enough to the defender go to ball, and we are behind it && we are close to our goal
  if((abs(robot_vec(0) - ball_vec(0)) < 1) && (robot_vec(0) > ball_vec(0)))
  {
    return Skills::goToPoint(robot, ball_vec);
  }
  //default defense is adaptiveRadiusGoalDefend
  return Skills::adaptiveRadiusGoalDefend(ally2_state_, ally1_state_, ball_state_);
}

State AI::play_standardOffense()
{
  State destination;
  typedef enum {
    GET_BALL, // Try to get control of the ball
    FIND_SHOT_SPOT, // Get away from opponents and get in position to shoot
    MAKE_SHOT // Take the shot on goal
  } state_t;
  static state_t play_state = GET_BALL;


  State robot_state;
  State ally_state;
  switch (robot_number_) {
    case 1:
      robot_state = ally1_state_;
      ally_state = ally2_state_;
      break;
    case 2:
      robot_state = ally2_state_;
      ally_state = ally1_state_;
      break;
    default:
      ROS_INFO("Invalid robot number in standardOffense() functions");
  }

  // State transitions

  // THESE ARE JUST TURNED OFF FOR TESTING! TODO: TURN BACK ON WHEN NEEDED!
  // if (!ballIsInPossessionOf(robot_state, ball_state_)) {
  //   play_state = GET_BALL;
  // }
  // else { // We have possession of the ball
  //   double shot_destination_y = Skills::findBestShot(ball_state_, ally_state, opp1_state_, opp2_state_);
  //   if(readyForGoalShot(shot_destination_y, robot_state, ally_state, opp1_state_, opp2_state_, ball_state_)){
  //     play_state = MAKE_SHOT;
  //   }
  //   else { // Not ready to shoot, keep moving/aligning
  //     play_state = FIND_SHOT_SPOT;
  //   }
  // }

  // Simple spot to test shooting
  Vector2d shot_spot_test(GOAL_X - 1.0, GOAL_Y + 0.5);
  State shot_spot_test_state = vectorToState(shot_spot_test);
  shot_spot_test_state.theta = atan2(GOAL_Y - shot_spot_test(1), GOAL_X - shot_spot_test(0));

  switch (play_state) {
    case GET_BALL:
      // Move towards the ball, aligning with path to shot spot
      destination = Skills::getBall(robot_state, ball_state_, shot_spot_test);

      break;

    case FIND_SHOT_SPOT:
      // Get ball out of opponent possession

      // Dribble ball towards shot spot and align with ball and goal
      // TODO: find the right angle to align with the best shot
      destination = shot_spot_test_state;

      break;

    case MAKE_SHOT:
      // Move forward towards ball until we are close enough to kick hard

      // Make a shot!

      break;

    default:

      break;
  }

  return destination;
}

State AI::play_skillsTournament(State robot_state) {
  int play_state = gameState_.home_score % 3;

  switch (play_state) {
    case 0:
      return Skills::spinInPlace(robot_state);

    case 1:
      return Skills::goToCenterFacingGoal();

    case 2:
      return Skills::moveInBoxFacingGoal(robot_state);
  }
}


void AI::stateCallback(const StateConstPtr &msg, const std::string& robot)
{
    if(robot == "ally1")
        ally1_state_ = *msg;

    else if(robot == "ally2")
        ally2_state_ = *msg;

    else if(robot == "opponent1")
        opp1_state_ = *msg;

    else if(robot == "opponent2")
        opp2_state_ = *msg;

    else if(robot == "ball") {
      ball_state_ = *msg;
    }

    computeDestination();
}


void AI::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
    gameState_ = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "home");
    ros::NodeHandle nh_;

    AI ai_node;

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
