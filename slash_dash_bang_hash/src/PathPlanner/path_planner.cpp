#include "PathPlanner/path_planner.h"
#include "Utilities/utilities.h"


#define BALL_RADIUS 0.022
#define AVOIDANCE_MARGIN 0.035

typedef Eigen::Matrix<double, 2, 4> Matrix2x4d;

PathPlanner::PathPlanner() :
nh_(ros::NodeHandle()),
priv_nh("~")
{
  // Private node handle to get whether we are home or away.
  // Having the nh_ private properly namespaces it.
  ros::NodeHandle priv_nh("~");

  max_xy_vel_ = priv_nh.param<double>("max_xy_vel_", 10.0); // m/s
  time_step_ = priv_nh.param<double>("time_step", 0.01); // 100 Hz
  robot_number_ = priv_nh.param<int>("robot_number", 1);


  ally1_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("ally1_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "ally1"));
  ally2_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("ally2_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "ally2"));
  opp1_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("opponent1_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "opponent1"));
  opp2_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("opponent2_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "opponent2"));
  ball_state_sub_  = nh_.subscribe<slash_dash_bang_hash::State>("ball_state", 1, boost::bind(&PathPlanner::stateCallback, this, _1, "ball"));

  destination_sub_ = nh_.subscribe<slash_dash_bang_hash::State>("destination", 1, &PathPlanner::destinationCallback, this);
  game_state_sub_ = nh_.subscribe<soccerref::GameState>("/game_state", 1, &PathPlanner::gameStateCallback, this);

  desired_pose_pub_ = nh_.advertise<slash_dash_bang_hash::State>("desired_pose", 5);


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


void PathPlanner::destinationCallback(const StateConstPtr &msg)
{
  //ROS_INFO("destination");

    destination_ = (*msg);
    planPath();
}


void PathPlanner::gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
  //ROS_INFO("game state call back");

    gameState_ = *msg;
}

void PathPlanner::planPath()
{
    // For now, just make the destination the desired pose
    desired_pose_ = destination_;

    // Updates desired_pose_
    // desired_pose_ = simpleCurvedPathToDestination();

    avoidBall(desired_pose_);

    publishDesiredPose();
}

void PathPlanner::publishDesiredPose()
{
    desired_pose_pub_.publish(desired_pose_);
}

State PathPlanner::avoidBall(State destination)
{
  typedef enum {
    AVOID_BALL, // Waypoint to side of the ball so we don't hit it backwards
    MOVE_TO_DESTINATION // Waypoint behind the ball
  } state_t;
  static state_t state = AVOID_BALL;

  State desired_pose;
  State robot_state;

  Vector2d robot_pose;
  if (robot_number_ == 1) {
    robot_state = ally1_state_;
  }
  else if (robot_number_ == 2) {
    robot_state = ally2_state_;
  }
  else {
    ROS_INFO("Invalid robot number in avoidBall() function!");
  }
  robot_pose << robot_state.x, robot_state.y;

  Vector2d ball_pose(ball_state_.x, ball_state_.y);

  Vector2d final_destination(destination.x, destination.y);

  // ==== Check if the ball is in the path ====
  Vector2d toDestination = final_destination - robot_pose;
  Vector2d perp(-1.0*toDestination(1), toDestination(0));
  perp = perp.normalized(); // Normalize the perpendicular vector
  Vector2d toBall = ball_pose - robot_pose;

  // Projection of ball onto the line perpendicular to the robot's movement
  double ball_perp_x = (toBall.dot(perp));

  // Check if we are facing the ball
  double ballAngle = fmod(atan2(toBall(1),toBall(0))*180/M_PI + 720.0, 360.0);
  double robotAngle = fmod(robot_state.theta + 720.0, 360.0);

  Vector2d robotForwardVec(cos(robot_state.theta*M_PI/180.0),sin(robot_state.theta*M_PI/180.0));

  double ballForwardDistance = robotForwardVec.dot(toBall)/robotForwardVec.norm();

  bool ballOnDestinationPath = (ballForwardDistance > 0.0);
  // ROS_INFO("Ball angle=%f, robot angle=%f", ballAngle, robot_state.theta);
  ROS_INFO("ballForwardDistance=%f", ballForwardDistance);

  // Assign appropriate state
  if ((abs(ball_perp_x) <= ROBOT_RADIUS + BALL_RADIUS)  && (toDestination.norm() > toBall.norm()) && !ballOnDestinationPath)
  {
    state = AVOID_BALL;
  }
  else {
    state = MOVE_TO_DESTINATION;
  }

  Vector2d avoidance_point;
  switch (state) {
    case AVOID_BALL:
      // Add intermediate destination to the side of the ball
      avoidance_point = ball_pose - (sgn(ball_perp_x)*(ROBOT_RADIUS + BALL_RADIUS + AVOIDANCE_MARGIN)*perp);
      desired_pose_.x = avoidance_point(0);
      desired_pose_.y = avoidance_point(1);
      break;

    case MOVE_TO_DESTINATION:
      desired_pose_= destination;
      break;

    default:
      ROS_INFO("Invalid state in avoidBall() function!");
  }

}

// Simple spline curve - no obstacle avoidance
State PathPlanner::simpleCurvedPathToDestination()
{
  // Calculate spline curve from initial/final pos & velocity
  Matrix2x4d endpoints;
    endpoints <<  ally1_state_.x, ally1_state_.xdot, destination_.x, cos(destination_.theta),
                  ally1_state_.y, ally1_state_.ydot, destination_.y, sin(destination_.theta);

  Eigen::Matrix4d coeff_transform;
    coeff_transform <<  1,  0, -3,  2,
                        0,  1, -2,  1,
                        0,  0,  3, -2,
                        0,  0, -1,  1;

  Matrix2x4d path_coeff = endpoints*coeff_transform;

  Vector2d robotToBall(ball_state_.x - ally1_state_.x, ball_state_.y - ally1_state_.y);
  double distance = robotToBall.norm();


  //Calculate "next point" on the path
  double tau = saturate((max_xy_vel_*time_step_)/distance, 0, 1.0);

  Vector4d Phi;
  Phi << 1.0, tau, tau*tau, tau*tau*tau;
  Vector2d point = path_coeff*Phi;

  // Calculate rotation
  Vector4d Phidot;
  Phidot << 0.0, 1.0, 2*tau, 3*tau*tau;
  Vector2d orientation = path_coeff*Phidot;

  State desired_pose;
  desired_pose.x = point(0);
  desired_pose.y = point(1);
  desired_pose.theta = angleMod(atan2(orientation(1), orientation(0))*180.0/M_PI);

  // Return next point
  return desired_pose;

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
