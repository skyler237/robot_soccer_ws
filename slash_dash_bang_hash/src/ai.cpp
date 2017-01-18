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

  ally1_goal_pub_ = nh.advertise<slash_dash_bang_hash::RobotState>("ally1_goal", 5);
  ally2_goal_pub_ = nh.advertise<slash_dash_bang_hash::RobotState>("ally2_goal", 5);

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

void AI::computeGoal() {
  double now = ros::Time::now().toSec();
  static double prev = 0;
  double dt = now - prev;
  prev = now;

  if (dt > AI_TIME_STEP)
  {
    if (gameState.play)
    {
        /*********************************************************************/
        // Choose strategies

        // robot #1 positions itself behind ball and rushes the goal.
        play_rushGoal(1, ally1, ball);

        // robot #2 stays on line, following the ball, facing the goal
        skill_followBallOnLine(2, ally2, ball, -2 * FIELD_WIDTH / 6);

        /*********************************************************************/
    }
    else if (gameState.reset_field)
    {
        skill_goToPoint(ally1, ally1_startingPos_, 1);
        skill_goToPoint(ally2, ally2_startingPos_, 2);
    }
    else //paused - do nothing
    {
        Vector3d zeroVel;
        zeroVel << 0, 0, 0;
        moveRobot(1, zeroVel);
        moveRobot(2, zeroVel);
    }
  }

  // TODO: Do we even need this??
  // Clean up
  Vector3d zeroVel;
  zeroVel << 0, 0, 0;
  moveRobot(1, zeroVel);
  moveRobot(2, zeroVel);
}


// play - rush goal
//   - go to position behind ball
//   - if ball is between robot and goal, go to goal
// NOTE:  This is a play because it is built on skills, and not control
// commands.  Skills are built on control commands.  A strategy would employ
// plays at a lower level.  For example, switching between offense and
// defense would be a strategy.
void AI::play_rushGoal(int robotId, RobotState robot, Vector2d ball)
{
    // normal vector from ball to goal
    Vector2d n = utility_unitVector(goal - ball);

    // compute position 10cm behind ball, but aligned with goal.
    Vector2d position = ball - 0.2*n;

    if(utility_vecLength(position - robot.pos) < 0.21)
        skill_goToPoint(robot, goal, robotId);
    else
        skill_goToPoint(robot, position, robotId);
}

void AI::visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg, const std::string& robot)
{
    if(robot == "ally1")
        ally1_state_ = utility_toRobotPose(*msg);

    else if(robot == "ally2")
        ally2_state_ = utility_toRobotPose(*msg);

    else if(robot == "opponent1")
        opp1_state_ = utility_toRobotPose(*msg);

    else if(robot == "opponent2")
        opp2_state_ = utility_toRobotPose(*msg);

    else if(robot == "ball")
        ball_state_ = utility_toBallPose(*msg);

    computeGoal();
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
