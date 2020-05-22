#!/usr/bin/env python

"""
    Copyright (C) 2019/2020 The University of Leeds and Mohammed Akram Fajer.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

# Python standard and System imports  
import sys 
import copy
from math import pi 

# ROS and Baxter SDK API imports
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped 
from std_msgs,.msg import String 
import baxter_interface
from baxter_interface import (
    CHECK_VERSION,
    Gripper,
    Limb
)
from baxter_core_msgs.msg import DigitalIOState 

# MoveIt imports
import moveit_commander 
import moveit_msgs.msg 
from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list

class BaxterManipulatorAndGripper:

    def __init__(self):
        self.left_limb  = None 
        self.right_limb = None

        self.left_gripper = None 
        self.right_gripper = None

        self.current_gripper = None  
        self.current_limb = None 

    def SetupBaxterManipualtor(self):
        self.left_limb = baxter_interface.limb.Limb('left')
        self.right_limb = baxter_interface.limb.Limb('right')
    
    def SetupBaxterGrippers(self):
        self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)

        self.left_gripper.calibrate()
        self.right_gripper.calibrate()
    
    def SetupBaxterChosenGripperAndManipulator(self, limb_name):
        self.current_gripper = baxter_interface.Gripper(limb_name, CHECK_VERSION)
        self.current_gripper.calibrate()
        self.current_limb = Limb(limb_name)

    def MoveBaxterManipulatorToNeutralPose(self):
        self.left_limb.move_to_neutral()
        self.right_limb.move_to_neutral()

    def OpenGripper(self, limb):
        if limb == 'left':
            self.left_gripper.open()
            rospy.sleep(1.0)
        elif limb == 'right':
            self.right_gripper.open()
            rospy.sleep(1.0)
     
    def CloseGripper(self, limb):
        if limb == 'left':
            self.left_gripper.close()
            rospy.sleep(1.0)
        elif limb == 'right':
            self.right_gripper.close()
            rospy.sleep(1.0)

    def OpenCurrentGripper(self):
        self.current_gripper.open(block=True)
    
    def CloseCurrentGripper(self):
        self.current_gripper.close(block=True)

class BaxterRobot:
    def __init__(self):
        self.robot_state = None 
        self.initial_state = None 

    def EnableBaxterRobot(self):
        self.robot_state = baxter_interface.RobotEnable(CHECK_VERSION)
        self.initial_state = self.robot_state.state().enabled

        if self.initial_state == False:
            rospy.loginfo("Enabling the robot")
            self.robot_state.enable()

    def DisableBaxterRobot(self):
        if self.initial_state == True:
            rospy.loginfo("Disabling the robot")
            self.robot_state.disable()

class Obstacle:
    
    """ This represents an obstacle in real world. """

    def __init__(self, obstacle_id='generic_obstacle', x=0.0, y=0.0, z=0.0, obstacle_shape_dimensions=(0.0, 0.0, 0.0)):

        """
            Initialize the obstacle attributes.

                An obstacle to be used with MoveIT motion planning framework and to plan trajectories that avoid
                colliding with it.

                An obstacle is defined by:
                    - Obstacle_id (i.e. name): is the name of the obstacle
                    - x, y, and z: is the obstacle 3D position relative to the world frame
                    - obstacle_shape_dimensions: is the obstacle model dimension (height, width and depth) 
        """

        self.name = obstacle_id

        # Obstacle 3D position vector
        self.pose = PoseStamped().pose.position = geometry_msgs.msg.Point( x, y, z )

        # The position vector relative to a specified frame in the scene that is dependent of the scene itself
        # Will be defined later
        self.pose.header.frame_id = None 
        self.size = obstacle_shape_dimensions

    def SetObstaclePoseParentFrameId(self, frame_id):
        
        """
            Will sent the parent frame id of the obstacle to determine the relative pose of an obstacle from
            the robot end-effector.
        """
        self.pose.header.frame_id = frame_id
    
class RobotSceneFactory:
    
    __scene_environment = None 

    @staticmethod
    def InitializeScene( CustomEnvironment ):
        RobotSceneFactory.__scene_environment = CustomEnvironment()
    
    @staticmethod
    def GetRobotSceneEnviroment():
        return RobotSceneFactory.__scene_environment.Clone()

class RobotScene:
    
    __obstacles = None 

    def Clone(self):
        pass 
    
    def GetObstacles(self):
        return self.__obstacles

class BaxterSimulationScene(RobotScene):
    
    def __init__(self):

        self.__obstacles = list()
    
    def __CreateObstacles(self):

        front_table = Obstacle(obstacle_id="front_table",
                x=1.0, y=0.0, z=0.0,
                obstacle_shape_dimensions=(0.913, 0.913, 0.04))
        
        self.__obstacles.append(front_table)
        
        left_side_table = Obstacle(obstacle_id="left_side_table",
                x=0.0, y=1.1, z=0.0,
                obstacle_shape_dimensions=(0.913, 0.913, 0.04))
        
        self.__obstacles.append(left_side_table)

        right_side_table = Obstacle(obstacle_id="right_side_table",
        x=0.0, y=-1.1, z=0.0,
        obstacle_shape_dimensions=(0.913, 0.913, 0.04))
        
        self.__obstacles.append(right_side_table)
    
    def Clone(self):
        return copy.deepcopy(self)

class MoveitPlanner:

    def __init__(self, manipulator_side = 'left'):

        # Enable the robot at start and disable at the end
        BaxterRobot().EnableBaxterRobot()
        rospy.on_shutdown(BaxterRobot().DisableBaxterRobot)

        # Test Open Both Grippers and Close Them 
        # baxter_manipulator = BaxterManipulatorAndGripper()
        
        # baxter_manipulator.SetupBaxterManipualtor()
        # baxter_manipulator.MoveBaxterManipulatorToNeutralPose()

        # baxter_manipulator.SetupBaxterGrippers()
        # baxter_manipulator.OpenGripper('left')
        # baxter_manipulator.OpenGripper('right')
        # rospy.sleep(3.0)
        # baxter_manipulator.CloseGripper('left')
        # baxter_manipulator.CloseGripper('right')

        # Referenceing baxter arm
        self.__manipulator_name = manipulator_side
        self.limb = MoveGroupCommander("{}_arm".format(manipulator_side))
        self.limb.set_end_effector_link("{}_gripper".format(manipulator_side))

        # Baxter Model configuration with MoveIt not configured to control its gripper,
        # thus using Baxter SDK API gripper class
        baxter_manipulator = BaxterManipulatorAndGripper()
        baxter_manipulator.SetupBaxterChosenGripperAndManipulator(manipulator_side)

        self.limb.set_planner_id("RRTConnectKConfigDefualt")
        self.limb.set_goal_position_tolerance(0.01)
        self.limb.set_goal_orientation_tolerance(0.01)


        


if __name__ == "__main__":
    
    rospy.init_node('moveit_planner', anonymous=True)

    try:
        MoveitPlanner()
        rospy.spin()
    except KeyboardInterrupt:
        pass 

