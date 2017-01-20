#include "PathPlanner/path_planner.h"


PathPlanner::PathPlanner() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");




  ally1_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("ally1_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "ally1"));
  ally2_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("ally2_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "ally2"));
  opp1_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("opponent1_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "opponent1"));
  opp2_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("opponent2_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "opponent2"));
  ball_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("ball_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "ball"));

  ally1_destination_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally1_destination", 1, boost::bind(&PathPlanner::destinationCallback, this, _1, "ally1"));
  ally2_destination_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("ally2_destination", 1, boost::bind(&PathPlanner::destinationCallback, this, _1, "ally2"));
  game_state_sub_ = nh_.subscribe<soccerref::GameState>("/game_state", 1, &PathPlanner::gameStateCallback, this);

  ally1_desired_pose_pub_ = nh_.advertise<slash_dash_bang_hash::State>("ally1_desired_pose", 5);
  ally2_desired_pose_pub_ = nh_.advertise<slash_dash_bang_hash::State>("ally2_desired_pose", 5);
  //ROS_INFO("Init");


}

void PathPlanner::stateCallback(const StateConstPtr &msg, const std::string& robot)
{
  //ROS_INFO("state call back");

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

}


void PathPlanner::destinationCallback(const StateConstPtr &msg, const std::string& robot)
{
  //ROS_INFO("destination");

    int robotId = 0;
    if(robot == "ally1") {
      ally1_destination_ = (*msg);
      robotId = 1;
    }
    else if(robot == "ally2") {
      ally2_destination_ = (*msg);
      robotId = 2;
    }


    planPath(robotId);
}


void PathPlanner::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
  //ROS_INFO("game state call back");

    gameState_ = *msg;
}

void PathPlanner::planPath(int robotId)
{
  //ROS_INFO("Plan path");
  switch(robotId)
  {
    case 1:
      // For now, just make the destination the desired pose
      ally1_desired_pose_ = ally1_destination_;
      break;

    case 2:
      ally2_desired_pose_ = ally2_destination_;
      break;

    default:
      ROS_INFO("ERROR: Invalid robot ID in planPath() function.");
      break;
  }
  // TODO: plan the path!!
  // Updates ally1_desired_pose_ and ally2_desired_pose_

  publishDesiredPose(robotId);
}

void PathPlanner::publishDesiredPose(int robotId)
{
  switch(robotId)
  {
    case 1:
      ally1_desired_pose_pub_.publish(ally1_desired_pose_);
      break;

    case 2:
      ally2_desired_pose_pub_.publish(ally2_desired_pose_);
      break;

    default:
      ROS_INFO("ERROR: Invalid robot ID in publishDesiredPose() function.");
      break;
  }
}


// -------------- Smoothed waypoint option --------------------
// Path_t InterceptController::getSmoothedWaypointPath(Eigen::Vector3d fleet_pos, Eigen::Vector3d target_pos, Eigen::Vector3d v_final) {
//   Path_t path;
//   int waypoint_cnt = 0;
//
//   if(v_final.norm() != 0) { // Only normalize if the vector is non-zero
//     v_final = v_final.normalized()*max_.velocity;
//   }
//   Matrix3x4d endpoints;
//   endpoints <<  fleet_pos(0), fleet_state_.xdot, target_pos(0), v_final(0),
//                 fleet_pos(1), fleet_state_.ydot, target_pos(1), v_final(1),
//                 fleet_pos(2), fleet_state_.zdot, target_pos(2), v_final(2);
//
//   // ROS_INFO("Enpoints matrix:");
//   // for(int i = 0; i < 3; i++) {
//   //   ROS_INFO("%f, %f, %f, %f", endpoints(i,0), endpoints(i,1), endpoints(i,2), endpoints(i,3));
//   // }
//
//
//   Eigen::Matrix4d coeff_transform;
//   coeff_transform <<  1,  0, -3,  2,
//                       0,  1, -2,  1,
//                       0,  0,  3, -2,
//                       0,  0, -1,  1;
//
//   Matrix3x4d path_coeff = endpoints*coeff_transform;
//
//   // Extract debug information
//   boost::array<double, 4ul> x_coeff = {path_coeff(0,0), path_coeff(0,1), path_coeff(0,2), path_coeff(0,3)};
//   boost::array<double, 4ul> y_coeff = {path_coeff(1,0), path_coeff(1,1), path_coeff(1,2), path_coeff(1,3)};
//   boost::array<double, 4ul> z_coeff = {path_coeff(2,0), path_coeff(2,1), path_coeff(2,2), path_coeff(2,3)};
//   debug_msg_.x_path_coeff = x_coeff;
//   debug_msg_.y_path_coeff = y_coeff;
//   debug_msg_.z_path_coeff = z_coeff;
//
//   Eigen::Vector4d Phi;
//   // Fill in path waypoints, by selecting points along the trajectory
//   for(int i = 0; i < max_.waypoint_cnt; i++) {
//     if(i >= MAX_WAYPOINT_CNT) {
//       ROS_INFO("WARNING: too many waypoints! Desired %d waypoints; max is %d waypoints", max_.waypoint_cnt, MAX_WAYPOINT_CNT);
//       break;
//     }
//     double tau = ((double) i)/(max_.waypoint_cnt - 1);
//     Phi << 1.0, tau, tau*tau, tau*tau*tau;
//     path.waypoints[i] = path_coeff*Phi;
//     // ROS_INFO("Tau = %f", tau);
//     // ROS_INFO("Phi: 0=%f, 1=%f, 2=%f, 3=%f", Phi(0), Phi(1), Phi(2), Phi(3));
//     // ROS_INFO("(pathPlanner) Waypoint %d: x=%f, y=%f, z=%f", i, path.waypoints[i](0), path.waypoints[i](1), path.waypoints[i](2));
//     waypoint_cnt++;
//   }
//   path.waypoint_cnt = waypoint_cnt;
//
//   return path;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "home");
    ros::NodeHandle nh_;

    PathPlanner path_planner_node;
    //ROS_INFO("main");

    ros::spin();
    //ROS_INFO("main 2");

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
