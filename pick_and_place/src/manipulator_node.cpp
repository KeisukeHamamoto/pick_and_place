#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <manipulator_node.hpp>
#include <geometry_msgs/Point.h>
#include <math.h>

manipulator_node::manipulator_node(std::string group_name)
{
    // move_start();
    move_group = new moveit::planning_interface::MoveGroupInterface(group_name);
    sub_ = nh.subscribe("/image_point", 10, &manipulator_node::move_point, this);
}

void manipulator_node::move_point(const geometry_msgs::Point::ConstPtr& image_point)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = move_group->getCurrentPose().pose;
    wpose.position.x = image_point->x;
    wpose.position.y = image_point->y;
    wpose.position.z = image_point->z;
    // wpose.orientation.x = 0.0;
    // wpose.orientation.y = 0.0;
    // wpose.orientation.z = 0.707106;
    // wpose.orientation.w = 0.707106;
    float roll = M_PI;
    float pich = M_PI_2;
    float yaw = M_PI_2;
    std::vector<std::vector<float> > x;
    x = {{cos(roll/2)},
         {sin(roll/2)},
         {0.0},
         {0.0}};
    std::vector<std::vector<float> > y;
    y = {{cos(pich/2)},
         {0.0},
         {sin(pich/2)},
         {0.0}};
    std::vector<std::vector<float> > z;
    z = {{cos(yaw/2)},
         {0.0},
         {0.0},
         {sin(yaw/2)}};
    std::vector<std::vector<float> > rpy;
    rpy = {{0.0},
           {0.0},
           {0.0},
           {0.0}};

    multiple_matrix(x, z, rpy);
    wpose.orientation.x = rpy[1][0];
    wpose.orientation.y = rpy[2][0];
    wpose.orientation.z = rpy[3][0];
    wpose.orientation.w = rpy[0][0];
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_thresold, trajectory);
    move_group->execute(trajectory);
}

void manipulator_node::multiple_matrix(std::vector<std::vector<float> > Matrix_1, std::vector<std::vector<float> > Matrix_2, std::vector<std::vector<float> > &ans)
{
    for (int i = 0; i < Matrix_1.size(); i++){
        for (int j = 0; j < Matrix_2[i].size(); j++){
            for(int k = 0; k < Matrix_2.size(); k++){
                ans[i][j] += Matrix_1[i][k] * Matrix_2[k][j];
            }
        }
    }
}

void manipulator_node::move_start()
{
    std::vector<geometry_msgs::Pose> home_pose;
    geometry_msgs::Pose home = move_group->getCurrentPose().pose;
    home.position.x = 0.0;
    home.position.y = 0.0;
    home.position.z = 1.2;
    // home.orientation.x = 0.0;
    // home.orientation.y = 0.0;
    // home.orientation.z = 0.707106;
    // home.orientation.w = 0.707106;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(home_pose, eef_step, jump_thresold, trajectory);
    move_group->execute(trajectory);
}