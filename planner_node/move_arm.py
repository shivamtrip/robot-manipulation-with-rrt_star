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
        # self.joint_control_client = actionlib.SimpleActionClient('/xarm/xarm6_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # self.joint_control_client.wait_for_server()
        # print("Joint Controller Client ready!")


    def publish_joint_trajectory(self):

        self.rate = rospy.Rate(1)  # Adjust the publishing rate as needed
        start_time = rospy.Time.now()

        # Create a FollowJointTrajectoryActionGoal message
        self.joint_goal = FollowJointTrajectoryActionGoal()

        # Set header
        self.joint_goal.header = Header()
        self.joint_goal.header.seq = 0
        self.joint_goal.header.stamp = start_time
        self.joint_goal.header.frame_id = ''

        # Set goal_id
        self.joint_goal.goal_id.stamp = start_time
        self.joint_goal.goal_id.id = "get_a_bottle"

        # Set goal trajectory
        self.joint_goal.goal.trajectory = JointTrajectory()
        self.joint_goal.goal.trajectory.header = Header()
        self.joint_goal.goal.trajectory.header.seq = 0
        self.joint_goal.goal.trajectory.header.stamp.secs = 0
        self.joint_goal.goal.trajectory.header.stamp.nsecs = 0
        self.joint_goal.goal.trajectory.header.frame_id = "world"
        self.joint_goal.goal.trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # Set trajectory points
        points = [[0,0,0,0,0,0]
                 [0,0.272636,-1.95957,0,1.68694,0],
                 [0,0.279253,-2.00713,0,1.72788,0]]
 
        
        # [[5.63897,0.823434,-1.34749,4.88605,0.0225471,4.22318],
        #         [3.49014,1.94488,-0.858725,2.00116,0.0197522,4.16485],
        #         [1.74883,-1.63055,-1.32914,0.0281994,-1.53398,0.469424],
        #         [3.1136,0.919679,-1.86924,0.428393,-1.57081,1.66126],
        #         [1.50931,-0.859195,-2.26143,0.749936,-1.32829,3.86032],
        #         [1.15838,-0.260147,-1.80895,2.07078,1.04048,4.46657],
        #         [1.14635,-0.239601,-1.79344,2.11609,1.12172,4.48737],
        #         ]
        
        # [
        #     [-9.371545261238623e-05, 0.0013225499395703721, 2.5108481410462957e-05, -0.0003216651907571588, 0.00014077304412118963, -2.1363318916556295e-05],
        #     [0.0320658560686014, 0.03700947829946987, -0.11720244258635941, 0.000262350719034969, 0.08228391901774308, 0.032253489644863735],
        #     [0.7074168580140909, 0.7864349738573594, -2.5789810150095267, 0.012526684824669653, 1.8072899844638026, 0.7100254018842499]
        # ]

        count = 1

        for pos in points:
            point = JointTrajectoryPoint()
            point.positions = pos #[0.7074168580140909, 0.7864349738573594, -2.5789810150095267, 0.012526684824669653, 1.8072899844638026, 0.7100254018842499]
            point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # Need to figure out how to set the appropriate time_from_start value for each point!
            point.time_from_start = rospy.Duration.from_sec(start_time.to_sec() + count)
            self.joint_goal.goal.trajectory.points.append(point)
            count += 1

        
        self.send_traj_goal()
        

    def send_traj_goal(self):

        # print(self.joint_goal)


        # self.joint_control_pub.publish(self.joint_goal)
        # # rospy.sleep(3)
        print("Goal published!")

        while not rospy.is_shutdown():
            self.joint_control_pub.publish(self.joint_goal)
            # self.joint_control_client.send_goal(self.joint_goal, self.dummy_cb)
            self.rate.sleep()


    def dummy_cb(self):
        pass

if __name__ == '__main__':

    robot_controller = MoveArm()
    
    try:
        robot_controller.publish_joint_trajectory()
    except rospy.ROSInterruptException:
        pass
