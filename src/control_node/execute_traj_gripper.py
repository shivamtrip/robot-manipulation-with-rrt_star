#!/usr/bin/env python3

import sys
# import copy
import rospy
import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

class MoveArm:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ctrl_guy')

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "xarm_gripper"
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
   
    def GoToWaypoint(self):
        try:
            rospy.logwarn("Closing Gripper")
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = 30*3.14/180
            self.move_group.go(joint_goal, wait=True)

        except rospy.ROSInterruptException:
            print("Failed to move to joint goal")
            pass

if __name__ == '__main__':
    Trajectory = []
    robot_controller = MoveArm()
    while not rospy.is_shutdown():
        robot_controller.GoToWaypoint()        
