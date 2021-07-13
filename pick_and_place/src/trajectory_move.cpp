#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#define PI 3.1415

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<trajectory_msgs::JointTrajectory>("/", 10);
    trajectory_msgs::JointTrajectoryPoint joint_value;
    joint_value.positions.push_back(-PI/2);
    joint_value.positions.push_back(-PI/2);
    joint_value.positions.push_back(-PI/5);
    joint_value.positions.push_back(-2*PI/3);
    joint_value.positions.push_back(PI/2);
    joint_value.positions.push_back(-PI/2);
    trajectory_msgs::JointTrajectory joint;
    joint.joint_names = {"shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint"};
    joint.points.push_back(joint_value);
    joint.header.stamp = ros::Time::now();

    pub.publish(joint);

    return 0;
}