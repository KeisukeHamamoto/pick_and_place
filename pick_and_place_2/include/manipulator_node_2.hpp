#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>
#include <shape_msgs/SolidPrimitive.h>

#include <string>
#include <vector>

class PickNPlacer {
    public:
        explicit PickNPlacer(ros::NodeHandle& node_handle);
        void DoPickAndPlace(geometry_msgs::Point::ConstPtr const &msg);
        void SetupPlanningScene();
        void AddBoxToScene(geometry_msgs::Point::ConstPtr const& msg);
        void RemoveBoxFromScene();
        bool DoPick(geometry_msgs::Point::ConstPtr const& msg);
        bool DoPlace();

    private:
    //Planning interface for the manipulator
    moveit::planning_interface::MoveGroupInterface manipulator_;
    //Planning interface for the gripper (used for planning scene purposes here)
    moveit::planning_interface::MoveGroupInterface gripper_group_;
    //Gripper control client
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
    //Object to manage the planning scene
    moveit::planning_interface::PlanningSceneInterface scene_;

    ros::Subscriber sub_;
};