#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_check");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    ros::Publisher planning_scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);


     // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene;
    planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

    // Define the pose of the table (transformed to robot's frame)
    geometry_msgs::Pose pose;
    pose.position.x = -0.32;
    pose.position.y = 0;
    pose.position.z = 0; //higher than actual
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;

    // Create a Collision Object
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "table";  // Unique ID for the object
    collision_object.header.frame_id = "link_base";  // Set the frame of your robot

    // Define the shape and size of the table (e.g., a box)
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.8;
    primitive.dimensions[1] = 1.5;
    primitive.dimensions[2] = 0.03;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the planning scene
    planning_scene->processCollisionObjectMsg(collision_object);

    // Update the planning scene
    planning_scene->getCurrentStateNonConst().update();

    // publish to display in rviz
    // moveit_msgs::PlanningScene planning_scene_msg;
    // planning_scene_pub.publish(planning_scene_msg);
    // ros::WallDuration sleep_time(1.0);
    // sleep_time.sleep();

    double joint1 = -121 * M_PI / 180.0; // Convert to radians
    double joint2 = 93 * M_PI / 180.0;
    double joint3 = -143 * M_PI / 180.0;
    double joint4 = -111 * M_PI / 180.0;
    double joint5 = 67 * M_PI / 180.0;
    double joint6 = -45 * M_PI / 180.0;
    std::vector<double> joint_values = {joint1, joint2, joint3, joint4, joint5, joint6}; // Replace with your joint values

    // Create a RobotState object for collision checking
    robot_state::RobotState current_state(kinematic_model);
    current_state.setJointGroupPositions("xarm6", joint_values);
    current_state.update(); // Update the robot state

    // Set up collision request and result
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = "xarm6"; // Specify the group name if needed
    collision_detection::CollisionResult collision_result;

    // Check for collisions with both the robot itself and the environment
    collision_request.contacts = true; // Optional: set to true if you want detailed contact information
    collision_request.max_contacts = 1000; // Optional: set maximum number of contacts to report

    // Check collision with the updated planning scene
    planning_scene->checkCollision(collision_request, collision_result, current_state);

    if (collision_result.collision) {
        ROS_INFO("Collision detected for the given joint configuration!");
        // return 0;
    } else {
        ROS_INFO("No collision detected for the given joint configuration.");
        // return 1;
    }

    while (ros::ok()) {
        // Add any additional code here related to action client, if needed
        // e.g., using SimpleActionClient to send the goal
        loop_rate.sleep();
    }

}