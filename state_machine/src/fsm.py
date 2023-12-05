
import rospy
from enum import Enum




class RobotStates(Enum):

    IDLE = -1
    PLANNING = 0
    MOVE_TO_PREGRASP = 1
    GRASP = 2
    MOVE_TO_PLACE = 3
    PLACE = 4
    MISSION_COMPLETE = 5


class Robot:

    def __init__(self, planner):

        self.state = RobotStates.IDLE
        self.planner = planner


    def update_robot_state(self, new_state : Enum):

        self.state = new_state



    def main(self):

        # Plan to pick up bottle
        self.state = RobotStates.PLANNING
        
        # planning function
        if not success:
            return False

        # Move to pregrasp pose 
        self.state = RobotStates.MOVE_TO_PREGRASP
        # control function
        if not success:
            return False        
    
        # Grasp bottle
        self.state = RobotStates.GRASP
        #  grasp function
        if not success:
            return False
        
        # Plan to place bottle
        self.state = RobotStates.PLANNING
        if not success:
            return False
        
        # Move to place pose
        self.state = RobotStates.MOVE_TO_PLACE
        if not success:
            return False
        
        # Place bottle
        self.state = RobotStates.PLACE
        if not success:
            return False
        
        # Mission Complete
        self.state = RobotStates.MISSION_COMPLETE
        return True


        

if __name__ == "__main__":

    robot = Robot("RRT")
    robot.main()



