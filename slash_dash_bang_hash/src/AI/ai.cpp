#include "AI/ai.h"
#include "Utilities/utilities.h"
#include "AI/skills.h"

#define AI_TIME_STEP 0.0

AI::AI() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");
  priv_nh.param<string>("team", team_, "home");

  ally1_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally1_state", 1, boost::bind(&AI::stateCallback, this, _1, "ally1"));
  ally2_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally2_state", 1, boost::bind(&AI::stateCallback, this, _1, "ally2"));
  opp1_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("opponent1_state", 1, boost::bind(&AI::stateCallback, this, _1, "opponent1"));
  opp2_state_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("opponent2_state", 1, boost::bind(&AI::stateCallback, this, _1, "opponent2"));
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
        ally1_destination_ = play_rushGoal(1, ally1_state_, ball_state_);
        //ROS_INFO("Ally1_destination: x=%f, y=%f", ally1_destination_.x, ally1_destination_.y);



        // robot #2 stays on line, following the ball, facing the goal
        ally2_destination_ = Skills::followBallOnLine(2, ally2_state_, ball_state_, -2 * FIELD_WIDTH / 6);

        ///////////////////////////////////////////////////////
        ////////////////////DEBUG//////////////////////////////
        //Vector2d dirGoal;
        //dirGoal << 1, 1;
        ///////////////////////////////////////////////////////
        //go to the center of the field, as a test to see if our controll is correct
        //ally2_destination_ = Skills::goToPoint(2, ally2_state_, dirGoal);
        //////////////////////////////////////////////////////////




        //ROS_INFO("Ally2_destination: x=%f, y=%f", ally2_destination_.x, ally2_destination_.y);
          //ROS_INFO("positionx: %d   positiony: %d", x_pos, ball.y);
        /*********************************************************************/
    }
    else if (gameState_.reset_field)
    {
        ally1_destination_ = Skills::goToPoint(1, ally1_state_, ally1_startingPos_);
        ally2_destination_ = Skills::goToPoint(2, ally2_state_, ally2_startingPos_);
    }
    else //paused - do nothing
    {
        Vector2d zeroVel;
        zeroVel << 0, 0;
        ally1_destination_.x = 0.0;
        ally1_destination_.y = 0.0;

        ally2_destination_.x = 0.0;
        ally2_destination_.y = 0.0;
    }
  }

  publishDestinations();
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
State AI::play_rushGoal(int robotId, State robot, State ball)
{
    // normal vector from ball to goal
    Vector2d ball_vec = stateToVector(ball);
    Vector2d n = (goal_ - ball_vec).normalized();

    // compute position 10cm behind ball, but aligned with goal.
    Vector2d position = ball_vec - 0.2*n;

    if((position - stateToVector(robot)).norm() < 0.21)
        return Skills::goToPoint(robotId, robot, goal_);
    else
        return Skills::goToPoint(robotId, robot, position);
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

    else if(robot == "ball")
        ball_state_ = *msg;

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
