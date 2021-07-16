#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <arm_test.hpp>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "arm_test");

    // ros::AsyncSpinner spinner(2);
    // spinner.start();
    manipulator_node mn("manipulator");

    // ros::waitForShutdown();
    // ros::shutdown();

    geometry_msgs::Point image_point;
    image_point.x = 0.00;
    image_point.y = 0.00;
    image_point.z = 1.50;

    mn.move_point(image_point);
}