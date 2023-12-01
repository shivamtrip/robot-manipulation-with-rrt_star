#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "collision_check");
        ros::NodeHandle nh_("~");

    // Load the robot model
    std::string group_name_ = "planning_scene";
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    const robot_model::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = group_name_;
    collision_request.distance = true;
    collision_detection::CollisionResult collision_result;
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startStateMonitor();

    if(planning_scene_monitor->getPlanningScene())
    {
    planning_scene_monitor->startSceneMonitor("/planning_scene");
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();
    }
    else
    {
    ROS_ERROR("Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
    }

    // Wait for initial messages
    ROS_INFO("Waiting for first joint msg.");
    ros::topic::waitForMessage<sensor_msgs::JointState>(parameters.joint_topic);
    ROS_INFO("Received first joint msg.");

    while (ros::ok())
    {
    // ** Update robot joints. This could happen in a callback function**
    pthread_mutex_lock(&shared_variables.joints_mutex);
    sensor_msgs::JointState jts = shared_variables.joints;
    pthread_mutex_unlock(&shared_variables.joints_mutex);

    for (std::size_t i = 0; i < jts.position.size(); ++i)
        current_state.setJointPositions(jts.name[i], &jts.position[i]);

    collision_result.clear();
    planning_scene_monitor->getPlanningScene()->checkCollision(collision_request, collision_result, current_state);
    }
}