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

import                      rospy
import                      tf
from std_msgs.msg import    Float64
import                      VectorUtils
import                      math
import copy
import numpy             as np

class Tf2Angles:

    def __init__(self, isMirroring = False, trackerName = 'tracker', baseFrame = 'tracker_depth_frame'):

        print( ' initializing Tf2Angles node ' )

        rospy.init_node( 'Tf2Angles', anonymous= True )

        self.__MIRROR               = isMirroring
        self.__publishers           = dict()
        self.__tf_transforms_names  = dict()
        self.__tracker_name         = trackerName + '/'
        self.__base_frame           = baseFrame
        self.__rate                 = rospy.Rate( 10 )
        self.__tf_listener          = tf.TransformListener()
        self.__math_utils           = VectorUtils.VectorUtilsNumpyWrapper()
        self.__smooth_angles        = dict()
        self.__smooth_range         = 15

        self.__setup_publishers()
        self.__setup_tf_transforms_names()
        self.__setup_smooth_angles( self.__smooth_range )

    def __setup_publishers( self ):

        print( ' from __setup_publishers() ' )

        # s0 -> shoulder roll
        # s1 -> shoulder pitch
        # e0 -> elbow roll
        # e1 -> elbow pitch

        arms = [ 'left', 'right' ]
        for arm in arms:
            self.__publishers[ arm + '_s0' ] = rospy.Publisher( arm + '_s0', Float64, queue_size= 10 )
            self.__publishers[ arm + '_s1' ] = rospy.Publisher( arm + '_s1', Float64, queue_size= 10 )
            self.__publishers[ arm + '_e0' ] = rospy.Publisher( arm + '_e0', Float64, queue_size= 10 )
            self.__publishers[ arm + '_e1' ] = rospy.Publisher( arm + '_e1', Float64, queue_size= 10 )

    def __setup_tf_transforms_names( self, userID = None ):

        print( ' from __setup_tf_transforms_names() ' )

        arms = [ 'left', 'right' ]
        user_id = '1' # default user

        if userID is not None:
            user_id = str ( userID )

        user = 'user_{}/'.format( user_id )

        for arm in arms:
            self.__tf_transforms_names[ arm + '_shoulder' ] = self.__tracker_name + user + arm + '_shoulder'

            if arm == 'left':
                self.__tf_transforms_names[ arm + '_mirrored_shoulder' ] = self.__tracker_name + user + 'right_shoulder'
            else:
                self.__tf_transforms_names[ arm + '_mirrored_shoulder' ] = self.__tracker_name + user + 'left_shoulder'

            self.__tf_transforms_names[ arm + '_elbow' ] = self.__tracker_name + user + arm + '_elbow'
            self.__tf_transforms_names[ arm + '_hand'  ] = self.__tracker_name + user + arm + '_hand'

            self.__tf_transforms_names[ arm + '_hip' ] = self.__tracker_name + user + arm + '_hip'

        self.__tf_transforms_names[ 'head'  ] = self.__tracker_name + user + 'head'
        self.__tf_transforms_names[ 'torso' ] = self.__tracker_name + user + 'torso'
        self.__tf_transforms_names[ 'base'  ] = self.__base_frame

    def __setup_smooth_angles( self, times = 20 ):

        print( ' from __setup_smooth_angles() ' )

        self.__smooth_angles[ 'left'  ] = [ [0] * times for i in range(4) ]
        self.__smooth_angles[ 'right' ] = [ [0] * times for i in range(4) ]

    def __get_transform( self, parent_frame, child_frame ):

        print( ' from __get_transform() ' )

        ( translation, rotation ) = self.__tf_listener.lookupTransform( parent_frame, child_frame, rospy.Time(0) )
        return translation

    def __get_user_id( self, frames ):

        print( ' from __get_user_id() ' )

        userid = str()
        for f in frames:
            temp = f.split('/')
            if len(temp) >= 3:
                userid = str(temp[1].split('_')[1])
                break
        return userid

    def __remove_unwanted_frames( self, frames, trackerName = 'tracker' ):

        print( ' from __remove_unwanted_frames() ' )

        result = list()
        for f in frames:
            temp = f.split('/')
            if len(temp) >= 3 and temp[0] == trackerName:
                result.append( f )
        return result

    def __convert2RobotCoordinates( self, vector ):
        c = copy.deepcopy( vector )
        return list( [ c[2], c[0],  c[1] ] )

    def __angle_adjustment( self, min, max, angle ):
            min_v = self.__math_utils.deg2rad( min )
            max_v = self.__math_utils.deg2rad( max )
            angle_v = copy.deepcopy( angle )
            if angle_v >= min_v:
                return min_v
            elif angle_v <= max_v:
                return max_v
            else:
                return angle_v

    def __compute_joint_angles( self, limb_name ):

        print( ' from __compute_joint_angles() ' )

        success = True

        try:
            # Transformation Frames relative to the base
            base = self.__tf_transforms_names[ 'base' ]

            base_2_shoulder             = list ( self.__get_transform( base, self.__tf_transforms_names[ limb_name + '_shoulder' ] ) )
            base_2_mirrored_shoulder    = list ( self.__get_transform( base, self.__tf_transforms_names[ limb_name + '_mirrored_shoulder' ] ) )
            base_2_elbow                = list ( self.__get_transform( base, self.__tf_transforms_names[ limb_name + '_elbow' ] ) )
            base_2_hand                 = list ( self.__get_transform( base, self.__tf_transforms_names[ limb_name + '_hand' ] ) )
            base_2_torso                = list ( self.__get_transform( base, self.__tf_transforms_names[ 'torso' ] ) )
            base_2_head                 = list ( self.__get_transform( base, self.__tf_transforms_names[ 'head'  ] ) )
            base_2_hip                  = list ( self.__get_transform( base, self.__tf_transforms_names[ limb_name + '_hip'] ) )

        except ( tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException ):
            success = False

        if success == True:
            print( ' tf success, will compute amgles ' )

            ''' Compute Vectors between upper body reference frames '''
            shoulder_2_elbow                = self.__math_utils.vector_subtraction_3D( base_2_elbow, base_2_shoulder )
            shoulder_2_hip                  = self.__math_utils.vector_subtraction_3D( base_2_hip, base_2_shoulder )
            elbow_2_hand                    = self.__math_utils.vector_subtraction_3D( base_2_hand, base_2_elbow )
            head_2_torso                    = self.__math_utils.vector_subtraction_3D( base_2_torso, base_2_head )
            shoulder_2_opposite_shoulder    = self.__math_utils.vector_subtraction_3D( base_2_mirrored_shoulder, base_2_shoulder )
            shoulder_2_hand                 = self.__math_utils.vector_subtraction_3D( base_2_hand, base_2_shoulder )

            ''' S0 Shoulder Yaw     '''
            C       = self.__math_utils.cross_product_3D( shoulder_2_elbow , head_2_torso )
            theta   = self.__math_utils.angle_between_two_vectors( C, shoulder_2_opposite_shoulder )

            if limb_name == 'right':
                angle_s0 = math.pi/4 - theta
            else:
                angle_s0 = 3*math.pi/4 - theta

            ''' S1 Shoulder Pitch   '''
            theta0 = self.__math_utils.angle_between_two_vectors( shoulder_2_hip,  shoulder_2_elbow )
            angle_s1 = math.pi/2 - theta0


            ''' E0 Shoulder Roll    '''
            C1 = self.__math_utils.cross_product_3D( shoulder_2_elbow, elbow_2_hand )
            C2 = self.__math_utils.cross_product_3D( shoulder_2_elbow , head_2_torso )
            theta1 = self.__math_utils.angle_between_two_vectors( C2, C1 )
            angle_e0 = theta1
            if limb_name == 'left':
                angle_e0 = -angle_e0

            ''' E1 Elbow Pitch      '''
            theta2 = self.__math_utils.angle_between_two_vectors( shoulder_2_elbow, elbow_2_hand )
            angle_e1 = theta2

            return { 's0': angle_s0, 's1': angle_s1, 'e0': angle_e0, 'e1': angle_e1 }

        else:
            print( ' tf exception, failed to find transformation ' )
            print( ' returning empty dictionary ' )
            return dict()

    def __track_arm( self, limb_name ):

        print( ' from __track_arm() ' )

        angles = self.__compute_joint_angles( limb_name )

        if angles == {}:
            print( ' tf listen failed ' )
            return

        if angles is not None:
            print( ' tf listen success, will smooth the data ' )

            for i in self.__smooth_angles[limb_name]:
                i.pop(0)

            self.__smooth_angles[limb_name][0].append( angles['s0'] )
            self.__smooth_angles[limb_name][1].append( angles['s1'] )
            self.__smooth_angles[limb_name][2].append( angles['e0'] )
            self.__smooth_angles[limb_name][3].append( angles['e1'] )

            command = [0.0, 0.0, 0.0, 0.0]
            for i in range(0, 4):
                sum = 0
                for j in range(0, self.__smooth_range):
                    sum = sum + self.__smooth_angles[limb_name][i][j]
                command[i] = sum / self.__smooth_range

            angle_s0 = command[0]
            angle_s1 = command[1]
            angle_e0 = command[2]
            angle_e1 = command[3]

            self.__publishers[ limb_name + '_s0' ].publish( angle_s0 )
            self.__publishers[ limb_name + '_s1' ].publish( angle_s1 )
            self.__publishers[ limb_name + '_e0' ].publish( angle_e0 )
            self.__publishers[ limb_name + '_e1' ].publish( angle_e1 )

    def run( self ):

        print( ' from run() ' )

        limb_1_name = 'left'
        limb_2_name = 'right'

        if self.__MIRROR == True:
            temp        = limb_1_name
            limb_1_name = limb_2_name
            limb_2_name = temp

        while not rospy.is_shutdown():

            userid      = str()
            base_frame  = 'tracker_depth_frame'
            foundUser   = False

            if self.__tf_listener.frameExists( base_frame ):

                print( ' found base frame ' )

                frames = self.__tf_listener.getFrameStrings()
                frames = self.__remove_unwanted_frames( frames )

                if len( frames ) > 0:
                    print( ' got frames names, now looking for user id ' )
                    userid = self.__get_user_id( frames )

                    if userid != str():
                        print( ' found user, with id ' + str(userid) )
                        foundUser = True
                    else:
                        print( ' failed to find user ' )
                        userid = str()
                        foundUser = False

                if foundUser == True:
                    print( ' user id is user_{} '.format( userid ) )
                    self.__setup_tf_transforms_names( userid )

                    print( ' about to track arms ' )
                    self.__track_arm( limb_1_name )
                    self.__track_arm( limb_2_name )

                    self.__rate.sleep()

            else:
                print( ' base frame not found ' )

if __name__ == "__main__":
    try:
        transform2angles = Tf2Angles()
        transform2angles.run()
        rospy.spin()
    except Exception as e:
        print(e)
