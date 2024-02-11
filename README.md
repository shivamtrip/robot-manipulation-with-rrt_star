# RRT-Star Planner to Manipulate Objects in Gazebo
 
- In this project, I implemented RRT-Star in C++ to plan a path towards an object grasp location in ROS Gazebo with a 6-DOF robotic arm. 

![2-ezgif com-video-to-gif-converter](https://github.com/shivamtrip/robot-manipulation-with-rrt_star/assets/66013750/444d1bae-3f6c-4e70-8c7e-0e7118424f42)


**Approach:**
1. Given a goal state (object 3D world coordinates) and an initial state (end-effector world coordinates), an optimal path is planned from start to goal using the RRT-Star algorithm.
2. The RRT-Star planner produces a collision-free path from the start to goal state.
3. Collision-checking is done by using the MoveIt! ROS package to validate whether a randomly sampled 3D coordinate is valid (collision-free) or not. MoveIt! internally uses the Flexible Collision Library (FCL) for collision-checking.
4. Once a valid path is found, a ROS message is sent to Gazebo which contains the complete trajectory for the 6-DOF robotic arm (joint angles at different time-steps).
5. Gazebo moves the end-effector (and arm) to a pre-grasp pose close to the object.
6. Next, a ROS message is sent to Gazebo to move the end-effector to grasp pose.
7. Finally, a ROS message is sent to Gazebo to close the gripper and grasp the object.    
