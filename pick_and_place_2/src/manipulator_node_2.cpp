#include <manipulator_node_2.hpp>

PickNPlacer::PickNPlacer(ros::NodeHandle& node_handle)
    :   manipulator_("manipulator"),
        gripper_group_("gripper"),
        gripper_("/gripper/gripper_cmd"){

        }

void PickNPlacer::SetupPlanningScene()
{
    ROS_INFO_STREAM("Setting up plnning scene");
    //Clear the planning scene
    std::vector<std::string> objs;
    for (auto o: scene_.getObjects()){
        objs.push_back(o.first);
    }
    for (auto o: scene_.getAttachedObjects()){
        objs.push_back(o.first);
    }
    scene_.removeCollisionObjects(objs);

    moveit_msgs::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "table";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.4;
    primitive.dimensions[2] = 0.4;
    geometry_msgs::Pose pose;
    pose.position.x = 0.3;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;
    std_msgs::ColorRGBA colour;
    colour.b = 0.5;
    colour.a = 1;
    scene_.applyCollisionObject(table, colour);

    //Let the planner know that this is the surface supporting we will
    //be picking and placing, so collisions are allowed
    manipulator_.setSupportSurfaceName("table");
}

bool PickNPlacer::DoPick(geometry_msgs::Pose2D::ConstPtr const& msg)
{
    //Prepare
    ROS_INFO_STREAM("Moving to box");
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 
}