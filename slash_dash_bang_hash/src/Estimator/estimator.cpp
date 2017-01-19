#include "slash_dash_bang_hash/estimator.h"


Estimator::Estimator() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh private properly namespaces it.
  ros::NodeHandle priv_nh("~");

  ally1_vision_sub_  = nh.subscribe<geometry_msgs::Pose2D>("ally1_vision", 1, boost::bind(visionCallback, _1, "ally1"));
  ally2_vision_sub_  = nh.subscribe<geometry_msgs::Pose2D>("ally2_vision", 1, boost::bind(visionCallback, _1, "ally2"));
  opp1_vision_sub_  = nh.subscribe<geometry_msgs::Pose2D>("opponent1_vision", 1, boost::bind(visionCallback, _1, "opponent1"));
  opp2_vision_sub_  = nh.subscribe<geometry_msgs::Pose2D>("opponent2_vision", 1, boost::bind(visionCallback, _1, "opponent2"));
  ball_vision_sub_  = nh.subscribe<geometry_msgs::Pose2D>("ball_vision", 1, boost::bind(visionCallback, _1, "ball"));

  ally1_state_pub_ = nh.advertise<slash_dash_bang_hash::State>("ally1_state", 5);
  ally2_state_pub_ = nh.advertise<slash_dash_bang_hash::State>("ally2_state", 5);
  opp1_state_pub_ = nh.advertise<slash_dash_bang_hash::State>("opp1_state", 5);
  opp2_state_pub_ = nh.advertise<slash_dash_bang_hash::State>("opp2_state", 5);
  ball_state_pub_ = nh.advertise<slash_dash_bang_hash::BallState>("ball_state", 5);

  game_state_sub_ = nh.subscribe<soccerref::GameState>("/game_state", 1, gameStateCallback);

}

void Estimator::visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg, const std::string& robot)
{
    if(robot == "ally1")
        ally1_vision_ = Utilities::poseToState(*msg);

    else if(robot == "ally2")
        ally2_vision_ = Utilities::poseToState(*msg);

    else if(robot == "opponent1")
        opp1_vision_ = Utilities::poseToState(*msg);

    else if(robot == "opponent2")
        opp2_vision_ = Utilities::poseToState(*msg);

    else if(robot == "ball")
        ball_vision_ = Utilities::poseToState(*msg);

    estimateStates();
}

void Estimator::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
    gameState_ = *msg;
}

void Estimator::estimateStates()
{
  // TODO: Actually implement an estimator here -- currently just passes the data through
  ally1_state_ = ally1_vision_;
  ally2_state_ = ally2_vision_;
  opp1_state_ = opp1_vision_;
  opp2_state_ = opp2_vision_;
  ball_state_ = ball_vision_;

  publishStates();
}

void Estimator::publishStates()
{
  ally1_state_pub_.publish(ally1_state_);
  ally2_state_pub_.publish(ally2_state_);
  opp1_state_pub_.publish(opp1_state_);
  opp2_state_pub_.publish(opp2_state_);
  ball_state_pub_.publish(ball_state_);
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
