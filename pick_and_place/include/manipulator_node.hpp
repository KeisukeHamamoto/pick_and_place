#pragma once
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/Grasp.h>

#define PI 3.1415
class manipulator_node
{
public:
    void move_point(const geometry_msgs::Point::ConstPtr& image_point);
    void multiple_matrix(std::vector<std::vector<float> > Matrix_1, std::vector<std::vector<float> > Matrix_2, std::vector<std::vector<float> > &ans);
    manipulator_node(std::string);
    moveit::planning_interface::MoveGroupInterface *move_group;
    geometry_msgs::Point box_point;
    ros::NodeHandle nh;
    ros::Subscriber sub_;
};