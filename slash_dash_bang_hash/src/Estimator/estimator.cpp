#include "Estimator/estimator.h"
#include "Utilities/utilities.h"


Estimator::Estimator() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");
  priv_nh.param<string>("team", team_, "home");
                                                            //topic  //queue size //callback function                 //shared pointer
  ally1_vision_sub_  = nh_.subscribe<geometry_msgs::Pose2D>("ally1_vision", 1, boost::bind(&Estimator::visionCallback, this, _1, "ally1"));
  ally2_vision_sub_  = nh_.subscribe<geometry_msgs::Pose2D>("ally2_vision", 1, boost::bind(&Estimator::visionCallback, this, _1, "ally2"));
  opp1_vision_sub_  = nh_.subscribe<geometry_msgs::Pose2D>("opponent1_vision", 1, boost::bind(&Estimator::visionCallback, this, _1, "opponent1"));
  opp2_vision_sub_  = nh_.subscribe<geometry_msgs::Pose2D>("opponent2_vision", 1, boost::bind(&Estimator::visionCallback, this, _1, "opponent2"));
  ball_vision_sub_  = nh_.subscribe<geometry_msgs::Pose2D>("ball_vision", 1, boost::bind(&Estimator::visionCallback, this, _1, "ball"));

  ally1_state_pub_ = nh_.advertise<slash_dash_bang_hash::State>("ally1_state", 5);
  ally2_state_pub_ = nh_.advertise<slash_dash_bang_hash::State>("ally2_state", 5);
  opp1_state_pub_ = nh_.advertise<slash_dash_bang_hash::State>("opp1_state", 5);
  opp2_state_pub_ = nh_.advertise<slash_dash_bang_hash::State>("opp2_state", 5);
  ball_state_pub_ = nh_.advertise<slash_dash_bang_hash::State>("ball_state", 5);

  game_state_sub_ = nh_.subscribe<soccerref::GameState>("/game_state", 1, &Estimator::gameStateCallback, this);

}

void Estimator::visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg, const std::string& robot)
{
    if(robot == "ally1")
        ally1_vision_ = poseToState(*msg);

    else if(robot == "ally2")
        ally2_vision_ = poseToState(*msg);

    else if(robot == "opponent1")
        opp1_vision_ = poseToState(*msg);

    else if(robot == "opponent2")
        opp2_vision_ = poseToState(*msg);

    else if(robot == "ball")
        ball_vision_ = poseToState(*msg);

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

slash_dash_bang_hash::State Estimator::poseToState(geometry_msgs::Pose2D pose)
{
  slash_dash_bang_hash::State state;
  // Flip coordinates if team is away or if we've swapped sides
  if((team_ == "away") ^ gameState_.second_half)
  {
      state.x = -pose.x;
      state.y = -pose.y;
      state.theta = angleMod(pose.theta + M_PI);
  }

  return state;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "home");
    ros::NodeHandle nh_;


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
