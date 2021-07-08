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

manipulator_node::manipulator_node(std::string group_name)
{
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
    // wpose.orientation.y = 0.707106;
    // wpose.orientation.z = 0.0;
    wpose.orientation.w = 1.0;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_thresold, trajectory);
    move_group->execute(trajectory);
}