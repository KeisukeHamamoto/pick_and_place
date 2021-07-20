#include <manipulator_node_2.hpp>

PickNPlacer::PickNPlacer(ros::NodeHandle& node_handle)
    :   manipulator_("manipulator"),
        gripper_group_("gripper"),
        gripper_("/gripper/gripper_cmd", "true"){
            manipulator_.setPoseReferenceFrame("base_link");
            gripper_.waitForServer();

            // Initialise the planning scene with known objects
            SetupPlanningScene();
            // Start by moving to the home pose
            manipulator_.setNamedTarget("home");
            manipulator_.move();

            sub_ = node_handle.subscribe("/image_point", 10, &PickNPlacer::DoPickAndPlace, this);
        }

void PickNPlacer::DoPickAndPlace(geometry_msgs::Point::ConstPtr const &msg)
{
    AddBoxToScene(msg);

    if(DoPick(msg)){
        DoPlace();
    }

    RemoveBoxFromScene();
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
    pose.position.z = -0.3;
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

void PickNPlacer::AddBoxToScene(geometry_msgs::Point::ConstPtr const& msg)
{
    ROS_INFO_STREAM("Adding box to planning scene at " << msg->x << "," << msg->y << "," << msg->z);
    moveit_msgs::CollisionObject redbox;
    redbox.header.frame_id = "base_link";
    redbox.id = "redbox";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.03;
    primitive.dimensions[1] = 0.03;
    primitive.dimensions[2] = 0.02;
    geometry_msgs::Pose pose;
    pose.position.x = msg->x;
    pose.position.y = msg->y;
    pose.position.z = -0.1;
    pose.orientation.w = 1.0;
    redbox.primitives.push_back(primitive);
    redbox.primitive_poses.push_back(pose);
    redbox.operation = redbox.ADD;
    scene_.applyCollisionObject(redbox);

    ros::Duration(1).sleep();
}

void PickNPlacer::RemoveBoxFromScene()
{
    ROS_INFO_STREAM("Removing box from planning scene");
    std::vector<std::string> objs;
    objs.push_back("redbox");
    scene_.removeCollisionObjects(objs);
}
bool PickNPlacer::DoPick(geometry_msgs::Point::ConstPtr const& msg)
{
    std::vector<moveit_msgs::Grasp> grasps;
    moveit_msgs::Grasp g;
    g.grasp_pose.header.frame_id = manipulator_.getPlanningFrame();
    g.grasp_pose.pose.position.x = msg->x;
    g.grasp_pose.pose.position.y = msg->y;
    g.grasp_pose.pose.position.z = msg->z + 0.1;
    g.grasp_pose.pose.orientation.w = 1;

    g.pre_grasp_approach.direction.header.frame_id = manipulator_.getPlanningFrame();
    g.pre_grasp_approach.direction.vector.z = -1;
    g.pre_grasp_approach.min_distance = 0.05;
    g.pre_grasp_approach.desired_distance = 0.07;

    g.post_grasp_retreat.direction.header.frame_id = manipulator_.getPlanningFrame();
    g.post_grasp_retreat.direction.vector.z = 1;
    g.post_grasp_retreat.min_distance = 0.05;
    g.post_grasp_retreat.desired_distance = 0.07;

    //grasping
    g.pre_grasp_posture.joint_names.resize(1, "finger1_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(1);
    g.pre_grasp_posture.points[0].positions[0] = 0.1;

    g.grasp_posture.joint_names.resize(1, "finger1_joint");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(1);
    g.grasp_posture.points[0].positions[0] = 0.01;

    grasps.push_back(g);

    manipulator_.setSupportSurfaceName("table");
    ROS_INFO_STREAM("Beginning pick");
    // Execute the pick
    if(!manipulator_.pick("redbox", grasps)){
        ROS_WARN("Pick failed");
        return false;
    }
    ROS_INFO_STREAM("Pick complete");
    return true;
}

bool PickNPlacer::DoPlace()
{
    std::vector<moveit_msgs::PlaceLocation> location;
    moveit_msgs::PlaceLocation p;
    p.place_pose.header.frame_id = manipulator_.getPlanningFrame();
    p.place_pose.pose.position.x = 0.0;
    p.place_pose.pose.position.y = 0.3;
    p.place_pose.pose.position.z = 0.4;
    p.place_pose.pose.orientation.w = 1.0;

    p.pre_place_approach.direction.header.frame_id = manipulator_.getPlanningFrame();
    p.pre_place_approach.direction.vector.z = -1;
    p.pre_place_approach.min_distance = 0.05;
    p.pre_place_approach.desired_distance = 0.07;

    p.post_place_posture.joint_names.resize(1, "finger1_joint");
    p.post_place_posture.points.resize(1);
    p.post_place_posture.points[0].positions.resize(1);
    p.post_place_posture.points[0].positions[0] = 0.1;

    location.push_back(p);

    manipulator_.setSupportSurfaceName("table");
    ROS_INFO_STREAM("Befinning place");

    manipulator_.place("redbox", location);
    ROS_INFO_STREAM("Place done");
    return true;
}