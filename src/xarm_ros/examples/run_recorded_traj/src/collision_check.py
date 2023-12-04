#!/usr/bin/env python3

import rospy
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import RobotState, Constraints
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
import math


class CollisionChecker:
    def __init__(self):
        rospy.init_node('moveit_scene_interface')
        self.robot = RobotCommander()
        self.group = MoveGroupCommander('xarm6')
        self.scene = PlanningSceneInterface()
        self.robot_state = RobotState()
        self.state_validity_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

    def send_req(self):
        joint_values = self.group.get_current_joint_values()
        # joint_values = [0,0,11*math.pi/180,0,0,0]
        joint_names = self.group.get_active_joints()
        self.robot_state.joint_state.position = joint_values
        self.robot_state.joint_state.name = joint_names
        request = GetStateValidityRequest()
        request.robot_state = self.robot_state
        request.group_name = 'xarm6'
        request.constraints = Constraints()
        response = self.state_validity_service(request)

        if response.valid:
            print("The joint state is valid and not in self-collision.")
        else:
            print("The joint state is not valid or is in self-collision.")


if __name__=='__main__':
    cc = CollisionChecker()
    cc.send_req()



