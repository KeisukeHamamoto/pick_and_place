#include <ros/ros.h>
#include <geometry_msgs/Point.h>

void callback(const geometry_msgs::Point image_point)
{
    ROS_INFO_STREAM("x:" << image_point.x);
    ROS_INFO_STREAM("y:" << image_point.y);
    ROS_INFO_STREAM("z:" << image_point.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/image_point", 10, callback);
    ros::spin();
    return 0;
}