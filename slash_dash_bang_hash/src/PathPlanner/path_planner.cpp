#include "slash_dash_bang_hash/path_planner.h"


PathPlanner::PathPlanner() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh private properly namespaces it.
  ros::NodeHandle priv_nh("~");
  priv_nh.param<string>("team", team, "home");

  ally1_destination_sub_ = nh.subscribe<geometry_msgs::Pose2D>("ally1_destination", 1, boost::bind(destinationCallback, _1, "ally1"));
  ally2_destination_sub_ = nh.subscribe<geometry_msgs::Pose2D>("ally2_destination", 1, boost::bind(destinationCallback, _1, "ally2"));
  game_state_sub_ = nh.subscribe<soccerref::GameState>("/game_state", 1, gameStateCallback);

  ally1_goal_pub_ = nh.advertise<slash_dash_bang_hash::RobotState>("ally1_goal", 5);
  ally2_goal_pub_ = nh.advertise<slash_dash_bang_hash::RobotState>("ally2_goal", 5);

  // This is sort of ad-hoc (would be much better to be a parameter) but it works for now
  ally1_startingPos_.x = -0.5;
  ally1_startingPos_.y = -0.0;

  ally2_startingPos_.x = -1.0;
  ally2_startingPos_.y = -0.0;

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
