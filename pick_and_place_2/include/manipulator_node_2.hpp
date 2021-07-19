#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <moveit_msgs/CollisionObject.h>
#include <control_msgs/GripperCommandAction.h>

#include <string>
#include <vector>

class PickNPlacer {
    public:
        explicit PickNPlacer(ros::NodeHandle& node_handle);
        void SetupPlanningScene();
        bool DoPick(geometry_msgs::Pose2D::ConstPtr const& msg);

    private:
    //Planning interface for the manipulator
    moveit::planning_interface::MoveGroupInterface manipulator_;
    //Planning interface for the gripper (used for planning scene purposes here)
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    //Gripper control client
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
    //Object to manage the planning scene
    moveit::planning_interface::PlanningSceneInterface scene_;
    std::string scene_task_frame_;
};