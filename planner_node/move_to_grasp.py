#!/usr/bin/env python

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, Duration
import time
import actionlib

class MoveArm:

    def __init__(self):

        rospy.init_node('joint_trajectory_publisher', anonymous=True)

        self.joint_control_pub = rospy.Publisher('/xarm/xarm6_traj_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
        # self.joint_control_client = actionlib.ActionClient('/xarm/xarm6_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    def publish_joint_trajectory(self):

        rate = rospy.Rate(1)  # Adjust the publishing rate as needed
        start_time = rospy.Time.now()

        # Create a FollowJointTrajectoryActionGoal message
        joint_goal = FollowJointTrajectoryActionGoal()

        # Set header
        joint_goal.header = Header()
        joint_goal.header.seq = 1
        joint_goal.header.stamp = start_time
        joint_goal.header.frame_id = ''

        # Set goal_id
        joint_goal.goal_id.stamp = start_time
        joint_goal.goal_id.id = "get_a_bottle"

        # Set goal trajectory
        joint_goal.goal.trajectory = JointTrajectory()
        joint_goal.goal.trajectory.header = Header()
        joint_goal.goal.trajectory.header.seq = 0
        joint_goal.goal.trajectory.header.stamp.secs = 0
        joint_goal.goal.trajectory.header.stamp.nsecs = 0
        joint_goal.goal.trajectory.header.frame_id = "world"
        joint_goal.goal.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # Set trajectory points
        points = [
            [0.2874493757963288, 0.6197583942477536, -2.332402190292693, -0.0007575067069822694, 1.7142849725813676, 0.2870580334845192],
            [0.37909797455755034, 0.6558056980065312, -2.3313896837679846, -0.0005881642725461312, 1.6772435621002775, 0.37871871317667505],
            [0.4707465733187719, 0.6918530017653088, -2.330377177243276, -0.0004188218381099931, 1.6402021516191876, 0.4703793928688309],
            [0.5623951720799933, 0.7279003055240864, -2.329364670718568, -0.0002494794036738549, 1.6031607411380975, 0.5620400725609868],
            [0.654043770841215, 0.763947609282864, -2.328352164193859, -8.013696923771674e-05, 1.5661193306570076, 0.6537007522531426],
            [0.7456923696024365, 0.7999949130416416, -2.327339657669151, 8.920546519842142e-05, 1.5290779201759175, 0.7453614319452985]
        ]

        for pos in points:
            point = JointTrajectoryPoint()
            point.positions = pos
            point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            point.accelerations = [1.0096451736637357, 0.0, 0.0, 0.0, 0.0, 0.0]
            point.time_from_start = rospy.Duration(rospy.Time.now().to_sec() - start_time.to_sec())
            joint_goal.goal.trajectory.points.append(point)

        while not rospy.is_shutdown():
            # self.joint_control_pub.publish(joint_goal)
            self.joint_control_client.send_goal(joint_goal, self.dummy_cb)
            rate.sleep()


    def dummy_cb(self):
        pass

if __name__ == '__main__':

    robot_controller = MoveArm()
    
    try:
        robot_controller.publish_joint_trajectory()
    except rospy.ROSInterruptException:
        pass
