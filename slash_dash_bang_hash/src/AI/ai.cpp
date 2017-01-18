#include "ai.h"
#include "utilities.h"

#define AI_TIME_STEP 0.0

AI::AI() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh private properly namespaces it.
  ros::NodeHandle priv_nh("~");
  priv_nh.param<string>("team", team, "home");

  ally1_vision_sub_ = nh.subscribe<geometry_msgs::Pose2D>("ally1_vision", 1, boost::bind(visionCallback, _1, "ally1"));
  ally2_vision_sub_ = nh.subscribe<geometry_msgs::Pose2D>("ally2_vision", 1, boost::bind(visionCallback, _1, "ally2"));
  opp1_vision_sub_ = nh.subscribe<geometry_msgs::Pose2D>("opponent1_vision", 1, boost::bind(visionCallback, _1, "opponent1"));
  opp2_vision_sub_ = nh.subscribe<geometry_msgs::Pose2D>("opponent2_vision", 1, boost::bind(visionCallback, _1, "opponent2"));
  ball_vision_sub_ = nh.subscribe<geometry_msgs::Pose2D>("ball_vision", 1, boost::bind(visionCallback, _1, "ball"));
  game_state_sub_ = nh.subscribe<soccerref::GameState>("/game_state", 1, gameStateCallback);

  ally1_destination_pub_ = nh.advertise<slash_dash_bang_hash::RobotState>("ally1_destination", 5);
  ally2_destination_pub_ = nh.advertise<slash_dash_bang_hash::RobotState>("ally2_destination", 5);

  // This is sort of ad-hoc (would be much better to be a parameter) but it works for now
  ally1_startingPos_.x = -0.5;
  ally1_startingPos_.y = -0.0;

  ally2_startingPos_.x = -1.0;
  ally2_startingPos_.y = -0.0;

}

void AI::param_init()
{
    goal_ << FIELD_WIDTH/2, 0;
}

void AI::computeDestination() {
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  if (dt > AI_TIME_STEP)
  {
    if (gameState.play)
    {
        /*********************************************************************/
        // Choose strategies and update ally1/ally2_destination_ variables

        // robot #1 positions itself behind ball and rushes the goal.
        ally1_destination_ = play_rushGoal(1, ally1, ball);

        // robot #2 stays on line, following the ball, facing the goal
        ally2_destination_ = Skills::followBallOnLine(2, ally2, ball, -2 * FIELD_WIDTH / 6);

        /*********************************************************************/
    }
    else if (gameState.reset_field)
    {
        ally1_destination_ = Skills::goToPoint(1, ally1, ally1_startingPos_);
        ally2_destination_ = Skills::goToPoint(2, ally2, ally2_startingPos_);
    }
    else //paused - do nothing
    {
        Vector2d zeroVel;
        zeroVel << 0, 0;
        ally1_destination_ = Utilities::vectorToRobotState(zeroVel);
        ally2_destination_ = Utilities::vectorToRobotState(zeroVel);
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
RobotState AI::play_rushGoal(int robotId, RobotState robot, Vector2d ball)
{
    // normal vector from ball to goal
    Vector2d ball_vec = Utiliies::robotStateToVector(robot);
    Vector2d n = (goal_ - ball_vec).normalized();

    // compute position 10cm behind ball, but aligned with goal.
    Vector2d position = ball_vec - 0.2*n;


    if((position - Utiliies::robotStateToVector(robot)).norm() < 0.21)
        return Skills::goToPoint(robotId, robot, goal);
    else
        return Skills::goToPoint(robotId, robot, position);
}

void AI::visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg, const std::string& robot)
{
    if(robot == "ally1")
        ally1_state_ = Utilities::poseToRobotState(*msg);

    else if(robot == "ally2")
        ally2_state_ = Utilities::poseToRobotState(*msg);

    else if(robot == "opponent1")
        opp1_state_ = Utilities::poseToRobotState(*msg);

    else if(robot == "opponent2")
        opp2_state_ = Utilities::poseToRobotState(*msg);

    else if(robot == "ball")
        ball_state_ = Utilities::poseToBallState(*msg);

    computeDestination();
}


void AI::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
    gameState_ = *msg;
}

int main(int argc, char **argv)
{
    param_init();
    ros::init(argc, argv, "home");
    ros::NodeHandle nh;


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
