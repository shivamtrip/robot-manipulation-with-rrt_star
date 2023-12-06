#!/bin/bash

# gnome-terminal -- bash -c "roslaunch spawn_object spawn_sdf.launch"

# wait

# roslaunch planner planner.launch

# wait

# # Wait for the first launch file to complete
# while rosnode list | grep -q planner_node; do
#     sleep 1
# done

# gnome-terminal -- bash -c "rosrun control_node move_to_grasp.py"


# # Wait for the first launch file to complete
# while rosnode list | grep -q control_node; do
#     sleep 1
# done

# gnome-terminal -- bash -c "rosrun control_node execute_traj_gripper.py"


# Launch the first roslaunch command and run it in the background
roslaunch spawn_object spawn_sdf.launch

# Launch the planner launch file
roslaunch planner planner.launch &

sleep 11

# Run the first control node
rosrun control_node move_to_grasp.py &

sleep 5

# Run the second control node
rosrun control_node execute_traj_gripper.py
