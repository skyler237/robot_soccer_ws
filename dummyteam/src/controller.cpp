#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "soccerref/GameState.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

#define ROBOT_MAX_VXY 2.0
#define ROBOT_MAX_OMEGA 2*M_PI
#define CONTROL_K_XY 5
#define CONTROL_K_OMEGA 2
#define FIELD_WIDTH 3.40  // in meters
#define FIELD_HEIGHT 2.38
#define ROBOT_RADIUS 0.10

struct RobotPose
{
    Vector2d pos;
    double theta;
};

string team;
Vector2d goal;
ros::Publisher motor_pub1;
ros::Publisher motor_pub2;
ros::Subscriber vsub_ally1, vsub_ally2;
ros::Subscriber vsub_opp1, vsub_opp2;
ros::Subscriber vsub_ball;
ros::Subscriber game_state_sub;
soccerref::GameState gameState;

RobotPose ally1, ally2;
RobotPose opp1, opp2;
Vector2d ball;
Vector2d ally1_startingPos;
Vector2d ally2_startingPos;

void moveRobot(Vector3d v_world, int robotId)
{
    geometry_msgs::Twist v;
    v.linear.x = v_world(0);
    v.linear.y = v_world(1);
    v.angular.z = v_world(2);

    if(robotId == 1)
        motor_pub1.publish(v);
    else if(robotId == 2)
        motor_pub2.publish(v);
}

void param_init()
{
    goal << FIELD_WIDTH/2, 0;
}

int utility_sgn(double val)
{
    return (0 < val) - (val < 0);
}

// utility - saturate velocity
//  Saturate the commanded velocity .
Vector3d utility_saturateVelocity(Vector3d v)
{
    if(fabs(v(0)) > ROBOT_MAX_VXY)
        v(0) = utility_sgn(v(0)) * ROBOT_MAX_VXY;
    if(fabs(v(1)) > ROBOT_MAX_VXY)
        v(1) = utility_sgn(v(1)) * ROBOT_MAX_VXY;
    if(fabs(v(2)) > ROBOT_MAX_OMEGA)
        v(2) = utility_sgn(v(2)) * ROBOT_MAX_OMEGA;
    return v;
}

double utility_vecLength(Vector2d v)
{
    return sqrt(v(0)*v(0) + v(1)*v(1));
}

Vector2d utility_unitVector(Vector2d v)
{
    return v / utility_vecLength(v);
}

double utility_angleMod(double angle)
{
    while(angle < 0)
        angle += 2*M_PI;
    return fmod(angle + M_PI, (2*M_PI)) - M_PI;
}

RobotPose utility_toRobotPose(Pose2D robot)
{
    // Flip coordinates if team is away or if we've swapped sides
    if((team == "away") ^ gameState.second_half)
    {
        robot.x = -robot.x;
        robot.y = -robot.y;
        robot.theta = utility_angleMod(robot.theta + M_PI);
    }
    Vector2d pos;
    pos << robot.x, robot.y;
    return (RobotPose){pos, robot.theta};
}

Vector2d utility_toBallPose(Pose2D ball)
{
    // Flip coordinates if team is away or if we've swapped sides
    if((team == "away") ^ gameState.second_half)
    {
        ball.x = -ball.x;
        ball.y = -ball.y;
    }
    Vector2d pos;
    pos << ball.x, ball.y;
    return pos;
}

// skill - follow ball on line
//   Follows the y-position of the ball, while maintaining x-position at x_pos.
//   Angle always faces the goal.
void skill_followBallOnLine(RobotPose robot, Vector2d ball, double x_pos, int robotId)
{
    // control x position to stay on current line
    double vx = CONTROL_K_XY * (x_pos - robot.pos(0));

    // control y position to match the ball's y-position
    double vy = CONTROL_K_XY * (ball(1) - robot.pos(1));

    // control angle to face the goal
    Vector2d dirGoal = goal - robot.pos;
    double theta_d = atan2(dirGoal(1), dirGoal(0));
    double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d);

    // Output velocities to motors
    Vector3d v;
    v << vx, vy, omega;
    v = utility_saturateVelocity(v);
    moveRobot(v, robotId);
}

// skill - go to point
//   Travels towards a point. Angle always faces the goal.
void skill_goToPoint(RobotPose robot, Vector2d point, int robotId)
{
    Vector2d dirPoint = point - robot.pos;
    Vector2d vxy = dirPoint * CONTROL_K_XY;

    // control angle to face the goal
    Vector2d dirGoal = goal - robot.pos;
    double theta_d = atan2(dirGoal(1), dirGoal(0));
    double omega = -CONTROL_K_OMEGA * (robot.theta - theta_d);

    // Output velocities to motors
    Vector3d v;
    v << vxy, omega;
    v = utility_saturateVelocity(v);
    moveRobot(v, robotId);
}

// play - rush goal
//   - go to position behind ball
//   - if ball is between robot and goal, go to goal
// NOTE:  This is a play because it is built on skills, and not control
// commands.  Skills are built on control commands.  A strategy would employ
// plays at a lower level.  For example, switching between offense and
// defense would be a strategy.
void play_rushGoal(RobotPose robot, Vector2d ball, int robotId)
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

void visionCallback(const geometry_msgs::Pose2D::ConstPtr &msg, const std::string& robot)
{
    if(robot == "ally1")
        ally1 = utility_toRobotPose(*msg);

    else if(robot == "ally2")
        ally2 = utility_toRobotPose(*msg);

    else if(robot == "opponent1")
        opp1 = utility_toRobotPose(*msg);

    else if(robot == "opponent2")
        opp2 = utility_toRobotPose(*msg);

    else if(robot == "ball")
        ball = utility_toBallPose(*msg);
}


void gameStateCallback(const soccerref::GameState::ConstPtr &msg)
{
    gameState = *msg;
}

int main(int argc, char **argv)
{
    param_init();
    ros::init(argc, argv, "home");
    ros::NodeHandle nh;

    // Private node handle to get whether we are home or away.
    // Having the nh private properly namespaces it.
    ros::NodeHandle priv_nh("~");
    priv_nh.param<string>("team", team, "home");

    vsub_ally1 = nh.subscribe<geometry_msgs::Pose2D>("ally1_vision", 1, boost::bind(visionCallback, _1, "ally1"));
    vsub_ally2 = nh.subscribe<geometry_msgs::Pose2D>("ally2_vision", 1, boost::bind(visionCallback, _1, "ally2"));
    vsub_opp1 = nh.subscribe<geometry_msgs::Pose2D>("opponent1_vision", 1, boost::bind(visionCallback, _1, "opponent1"));
    vsub_opp2 = nh.subscribe<geometry_msgs::Pose2D>("opponent2_vision", 1, boost::bind(visionCallback, _1, "opponent2"));
    vsub_ball = nh.subscribe<geometry_msgs::Pose2D>("ball_vision", 1, boost::bind(visionCallback, _1, "ball"));
    vsub_ball = nh.subscribe<geometry_msgs::Pose2D>("ball_vision", 1, boost::bind(visionCallback, _1, "ball"));
    game_state_sub = nh.subscribe<soccerref::GameState>("/game_state", 1, gameStateCallback);
    motor_pub1 = nh.advertise<geometry_msgs::Twist>("ally1/vel_cmds", 5);
    motor_pub2 = nh.advertise<geometry_msgs::Twist>("ally2/vel_cmds", 5);

    // This is sort of ad-hoc (would be much better to be a parameter) but it works for now
    ally1_startingPos << -0.5, 0;
    ally2_startingPos << -1.0, 0;

    Vector2d left_corner(-FIELD_WIDTH/2, FIELD_HEIGHT/2);
    Vector2d right_corner(-FIELD_WIDTH/2, -FIELD_HEIGHT/2);

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        if (gameState.play)
        {

            /*********************************************************************/
            // Choose strategies

            // robot #1 positions itself behind ball and rushes the goal.
            // skill_goToPoint(ally1, left_corner, 1);
            // play_rushGoal(ally1, ball, 1);

            // robot #2 stays on line, following the ball, facing the goal
            // skill_goToPoint(ally2, right_corner, 2);
            // skill_followBallOnLine(ally2, ball, -2 * FIELD_WIDTH / 6, 2);

            /*********************************************************************/
        }
        else if (gameState.reset_field)
        {
            skill_goToPoint(ally1, ally1_startingPos, 1);
            skill_goToPoint(ally2, ally2_startingPos, 2);
        }
        else //paused - do nothing
        {
            Vector3d zeroVel;
            zeroVel << 0, 0, 0;
            moveRobot(zeroVel, 1);
            moveRobot(zeroVel, 2);
        }

        // process any callbacks
        ros::spinOnce();

        // force looping at a constant rate
        loop_rate.sleep();
    }

    // Clean up
    Vector3d zeroVel;
    zeroVel << 0, 0, 0;
    moveRobot(zeroVel, 1);
    moveRobot(zeroVel, 2);
    return 0;
}
