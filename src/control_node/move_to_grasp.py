#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
# import moveit_msgs.msg
import geometry_msgs.msg
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

class MoveArm:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ctrl_guy')

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "xarm6"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        

        # display_trajectory_publisher = rospy.Publisher(
        #     "/move_group/display_planned_path",
        #     moveit_msgs.msg.DisplayTrajectory,
        #     queue_size=20,
        # )

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

                

                # The go command can be called with joint values, poses, or without any
                # parameters if you have already set the pose or joint target for the group
   
    def GoToWaypoint(self, Waypoint, index):
        try:
            rospy.logwarn("Moving to Waypoint: "+ str(index) + "/60")
            # We get the joint values from the group and change some of the values:
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = Waypoint[0] 
            joint_goal[1] = Waypoint[1]
            joint_goal[2] = Waypoint[2]
            joint_goal[3] = Waypoint[3]
            joint_goal[4] = Waypoint[4]
            joint_goal[5] = Waypoint[5]

            self.move_group.go(joint_goal, wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
            # self.move_group.stop()
        except rospy.ROSInterruptException:
            print("Failed to move to joint goal")
            pass


    def GoToCoordinates(self):
        
        waypoints = []

        current_pose = self.move_group.get_current_pose().pose
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.x = -0.707
        # pose_goal.orientation.y = 0
        # pose_goal.orientation.z = -0.707
        # pose_goal.orientation.w = 0
        # pose_goal.position.x = current_pose.position.x + 0.1
        # pose_goal.position.y = current_pose.position.y
        # pose_goal.position.z = current_pose.position.z
        # self.move_group.set_pose_target(pose_goal)
        current_pose.position.x -= 0.08
        waypoints.append(copy.deepcopy(current_pose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold


        self.move_group.execute(plan, wait=True)

if __name__ == '__main__':
    Trajectory = []
    robot_controller = MoveArm()
    robot_controller.GoToCoordinates()


    # while not rospy.is_shutdown():
        # joint_goal = robot_controller.move_group.get_current_joint_values()
        # for i in range (30):
        #     joint_goal[0] = joint_goal[0]
        #     joint_goal[1] = joint_goal[1]
        #     joint_goal[2] = joint_goal[2] - 1 * 3.14159 / 180
        #     joint_goal[3] = joint_goal[3] 
        #     joint_goal[4] = joint_goal[4]
        #     joint_goal[5] = joint_goal[5] 
        #     Trajectory.append([joint_goal[0], joint_goal[1], joint_goal[2], joint_goal[3], joint_goal[4], joint_goal[5]])
        # for j in range (30):
        #     joint_goal[0] = joint_goal[0]
        #     joint_goal[1] = joint_goal[1]
        #     joint_goal[2] = joint_goal[2] + 1 * 3.14159 / 180
        #     joint_goal[3] = joint_goal[3] 
        #     joint_goal[4] = joint_goal[4]
        #     joint_goal[5] = joint_goal[5] 
        #     Trajectory.append([joint_goal[0], joint_goal[1], joint_goal[2], joint_goal[3], joint_goal[4], joint_goal[5]])

        # for i in range(len(Trajectory)):
        #     robot_controller.GoToWaypoint(Trajectory[i], i)
        #     # rospy.sleep(0.1)
