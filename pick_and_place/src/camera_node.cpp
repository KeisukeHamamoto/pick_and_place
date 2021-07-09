#include <camera_node.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_node");
    camera_node cn;

    ros::spin();
    return 0;
}