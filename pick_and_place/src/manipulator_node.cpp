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
    // move_start();
    move_group = new moveit::planning_interface::MoveGroupInterface(group_name);
    sub_ = nh.subscribe("/image_point", 1000, &manipulator_node::move_point, this);
}

void manipulator_node::move_point(const geometry_msgs::Point::ConstPtr& image_point)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = move_group->getCurrentPose().pose;
    wpose.position.x = image_point->x;
    wpose.position.y = image_point->y;
    wpose.position.z = image_point->z;
    wpose.orientation.x = 0.0;
    wpose.orientation.y = 0.707106;
    wpose.orientation.z = 0.0;
    wpose.orientation.w = 0.707106;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_thresold, trajectory);
    move_group->execute(trajectory);
}

// void manipulator_node::move_start()
// {
//     ros::Publisher pub;
//     pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);
//     trajectory_msgs::JointTrajectoryPoint joint_value;
//     joint_value.positions.push_back(-PI/2);
//     joint_value.positions.push_back(-PI/2);
//     joint_value.positions.push_back(-PI/5);
//     joint_value.positions.push_back(-2*PI/3);
//     joint_value.positions.push_back(PI/2);
//     joint_value.positions.push_back(-PI/2);

//     trajectory_msgs::JointTrajectory joint;
//     joint.joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };

//     joint.header.stamp = ros::Time::now();

//     ros::Rate loop(10);
//     while (ros::ok()) {
//         pub.publish(joint);
//         loop.sleep();
//     }
// }