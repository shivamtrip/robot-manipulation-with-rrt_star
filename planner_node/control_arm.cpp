#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Header.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "control_arm");
    ros::NodeHandle nh;
    ros::Publisher traj_publisher = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/xarm/xarm6_traj_controller/follow_joint_trajectory/goal", 10);

    ros::Rate loop_rate(1);

    int count = 1;


    control_msgs::FollowJointTrajectoryActionGoal joint_goal;
    ros::Time start_time = ros::Time::now();

    // Set header
    joint_goal.header.seq = 0;
    joint_goal.header.stamp = start_time;
    joint_goal.header.frame_id = "";

    // Set goal_id
    joint_goal.goal_id.stamp = start_time;
    joint_goal.goal_id.id = "get_a_bottle";

    // Set goal trajectory
    joint_goal.goal.trajectory.header.seq = 0;
    joint_goal.goal.trajectory.header.stamp = ros::Time(0);
    joint_goal.goal.trajectory.header.frame_id = "world";
    joint_goal.goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    // Set trajectory points
    std::vector<std::vector<double>> points = {
        {0,0,0,0,0,0},
        {0,0.272636,-1.95957,0,1.68694,0},
        {0,0.279253,-2.00713,0,1.72788,0}
        }; 

    for (const auto &pos : points) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = pos;
        point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        point.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        point.time_from_start = ros::Duration(count); // Adjust the time_from_start as needed
        joint_goal.goal.trajectory.points.push_back(point);
        count++;
    }

    while (ros::ok()) {
        traj_publisher.publish(joint_goal);
        // Add any additional code here related to action client, if needed
        // e.g., using SimpleActionClient to send the goal
        loop_rate.sleep();
    }

  return 0;
}