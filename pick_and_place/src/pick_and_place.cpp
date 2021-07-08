#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <manipulator_node.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "pick_and_place");

    ros::AsyncSpinner spinner(2);
    spinner.start();
    manipulator_node mn("manipulator");

    ros::waitForShutdown();
    ros::shutdown();

    // geometry_msgs::Point image_point;
    // image_point.x = 0.30;
    // image_point.y = -0.1;
    // image_point.z = 0.8;

    // mn.move_point(image_point);
}