#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_check");
    ros::NodeHandle nh;

    // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // Create a RobotState object for collision checking
    robot_state::RobotState current_state(kinematic_model);
    
    // Set the joint values for your arbitrary joint configuration
    double joint1 = 0.0;
    double joint2 = 0.0;
    double joint3 = 0.0;
    double joint4 = 0.0;
    double joint5 = 0.0;
    double joint6 = 0.0;


    std::vector<double> joint_values = {joint1, joint2, joint3, joint4, joint5, joint6}; // Replace with your joint values
    current_state.setJointGroupPositions("arm", joint_values); // "arm" is the planning group name for your xArm6

    // Check for collisions
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkCollision(collision_request, collision_result, current_state);

    if (collision_result.collision)
    {
        ROS_ERROR("Collision detected for the given joint configuration!");
    }
    else
    {
        ROS_INFO("No collision detected for the given joint configuration.");
    }

    return 0;
}