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

import                          rospy
import                          roslib
import                          baxter_interface
from std_msgs.msg import        Float64
from baxter_interface import    CHECK_VERSION

class Tf2AnglesController:

    """
        Tf2AnglesController listen to the published human-robot mapped
        joint angles and control Baxter's joints.
    """

    def __init__( self ):

        print( ' initializing Tf2AnglesController node ' )

        rospy.init_node( 'Tf2AnglesController', anonymous= True )

        # Enable the Robot
        self.__robot                = baxter_interface.RobotEnable( CHECK_VERSION )

        # Robot Limbs
        self.__left_limb            = baxter_interface.limb.Limb( 'left' )
        self.__right_limb           = baxter_interface.limb.Limb( 'right' )

        self.__rate                 = rospy.Rate( 10 )
        self.__command_joint_angles = dict()

        self.__enable_robot_and_move_to_neutral_pose()
        self.__setup_subscribers()
        self.__setup_command_joint_angles()

    def __enable_robot_and_move_to_neutral_pose( self ):

        print( ' from __enable_robot_and_move_to_neutral_pose() ' )

        if not self.__robot.state().enabled:
            print( ' enabling the robot ' )
            self.__robot.enable()

        # Move robot arms to their neutral pose
        self.__left_limb.move_to_neutral()
        self.__right_limb.move_to_neutral()

    def __setup_subscribers( self ):

        print( ' from __setup_subscribers() ' )

        arms = [ 'left', 'right' ]

        for arm in arms:

            rospy.Subscriber( arm + '_s0', Float64, self.__human_2_robot_angles_callback( arm, 0 ) )
            rospy.Subscriber( arm + '_s1', Float64, self.__human_2_robot_angles_callback( arm, 1 ) )
            rospy.Subscriber( arm + '_e0', Float64, self.__human_2_robot_angles_callback( arm, 2 ) )
            rospy.Subscriber( arm + '_e1', Float64, self.__human_2_robot_angles_callback( arm, 3 ) )

    def __setup_command_joint_angles( self ):

        print( ' from __setup_command_joint_angles() ' )

        self.__command_joint_angles[ 'left'  ] = [ 0.0 for i in range(4) ]
        self.__command_joint_angles[ 'right' ] = [ 0.0 for i in range(4) ]

    def __human_2_robot_angles_callback( self, limb_name, index ):
        def call_back_function( f ):
            print( limb_name )
            print( index )
            print( self.__command_joint_angles )
            self.__command_joint_angles[limb_name][index] = f.data
        return call_back_function

    def run( self ):

        print( ' from run() ' )

        rospy.sleep( 5 )

        while not rospy.is_shutdown():

            angles = self.__left_limb.joint_angles()

            angles['left_s0'] = self.__command_joint_angles['left'][0]
            angles['left_s1'] = self.__command_joint_angles['left'][1]
            angles['left_e0'] = self.__command_joint_angles['left'][2]
            angles['left_e1'] = self.__command_joint_angles['left'][3]
            angles['left_w0'] = 0
            angles['left_w1'] = 0
            angles['left_w2'] = 0

            self.__left_limb.set_joint_positions( angles )

            angles = self.__right_limb.joint_angles()

            angles['right_s0'] = self.__command_joint_angles['right'][0]
            angles['right_s1'] = self.__command_joint_angles['right'][1]
            angles['right_e0'] = self.__command_joint_angles['right'][2]
            angles['right_e1'] = self.__command_joint_angles['right'][3]
            angles['right_w0'] = 0
            angles['right_w1'] = 0
            angles['right_w2'] = 0

            self.__right_limb.set_joint_positions( angles )

            self.__rate.sleep()

if __name__ == "__main__":

    try:

        transform2anglesController = Tf2AnglesController()
        transform2anglesController.run()

    except Exception as e:
        print( e )
