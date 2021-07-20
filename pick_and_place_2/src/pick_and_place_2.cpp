#include <manipulator_node_2.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_and_place_2");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh;
    PickNPlacer pnp(nh);

    ros::waitForShutdown();
    ros::shutdown();
    return 0;
}