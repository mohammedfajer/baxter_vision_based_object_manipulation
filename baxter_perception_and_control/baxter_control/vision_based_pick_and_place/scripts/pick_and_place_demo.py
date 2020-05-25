#!/usr/bin/env python

# Python standard imports
import sys
import time
import copy
from math import pi
import struct
from abc import ABCMeta, abstractmethod
import json
import numpy as np

# ROS and Baxter SDK imports
import rospy
import rospkg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from std_msgs.msg import (
    Header,
    Empty
)

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

import baxter_interface
from baxter_interface import  CHECK_VERSION, Gripper, Limb
import baxter_core_msgs.msg

# MoveIt! imports
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander

class RobotEndEffectorPose:
    """
    A class used to represent the end-effector pose
    (i.e. position and orientation) of the robot arm relative to the robot
    base.
    """

    def __init__(self, pos_x, pos_y, pos_z, roll, pitch, yaw):
        """
        Constructor.

        @param pos_x: represent the position along the x-axis of the end-effector.
        @param pos_y: represent the position along the y-axis of the end-effector.
        @param pos_z: represent the position along the z-axis of the end-effector.
        @param roll: represent the rotation along the x-axis of the end-effector.
        @param pitch: represent the rotation along the y-axis of the end-effector.
        @param yaw: represent the rotation along the z-axis of the end-effector.
        """

        self.position_x = pos_x
        self.position_y = pos_y
        self.position_z = pos_z

        self.rotation_x = roll
        self.rotation_y = pitch
        self.rotation_z = yaw

        self.time_created = time.time()

    def __str__(self):
        """String representation of the end-effector pose."""
        return 'x={} y={} z={} r={} p={} y={}'.format(self.position_x,
                                                      self.position_y,
                                                      self.position_z,

                                                      self.rotation_x,
                                                      self.rotation_y,
                                                      self.rotation_z)
    def __get_position_and_orientation(self):
        """ Return the pose components of the end-effector. """
        position = Point(self.position_x, self.position_y, self.position_y)
        q = quaternion_from_euler(self.rotation_x,
                                  self.rotation_y,
                                  self.rotation_z)
        orientation = Quaternion(q[0], q[1], q[2], q[3])
        return position, orientation

    def get_pose(self):
        """ Return the pose of the end-effector. """
        position, orientation = self.__get_position_and_orientation()
        return Pose(position=position, orientation=orientation)

    def get_pose_stamped(self):
        """ Return the pose stamped object of the end-effector pose."""
        pose = self.get_pose()
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        return PoseStamped(header=header, pose=pose)

    def is_empty(self):
        """ Return boolean value indicating the state of the pose. """
        pose_components=list([self.position_x,
                              self.position_y,
                              self.position_z,

                              self.rotation_x,
                              self.rotation_y,
                              self.rotation_z])

        return all(map(lambda v: v == 0, pose_components))

class RobotArm:
    """
    A class used to represent the arm of a robot
    using both "Moveit" API and Baxter SDK API.
    """

    def __init__(self, side_name):
        """
        Constructor.

        @param side_name: which arm side we are refering to 'left' or 'right'.
        """

        self.__side_name = side_name

        # MoveIt interface to a group of arm joints.
        # Either left arm joint group or right arm joint group.
        self.moveit_limb = MoveGroupCommander('{}_arm'.format(side_name))

        # MoveIt limb setting.
        self.moveit_limb.set_end_effector_link('{}_gripper'.format(side_name))
        self.moveit_limb.set_planner_id('RRTConnectKConfigDefault')
        self.moveit_limb.set_goal_position_tolerance(0.01)
        self.moveit_limb.set_goal_orientation_tolerance(0.01)

        # MoveIt does not provide support for Baxter gripper.
        # Thus we use Baxter SDK gripper instead.
        self.gripper = Gripper(side_name, CHECK_VERSION)
        self.gripper.calibrate()

        self.baxter_sdk_limb = Limb(side_name)

    def __str__(self):
        """
        Built-in function to return a printable string
        representing the arm used in either print(object)
        or str(object).
        """
        return str(self.__side_name)

    def open_gripper(self):
        """
        Command the robot arm end-effector i.e. the gripper to open.
        """
        self.gripper.open(block=True)

    def close_gripper(self):
        """
        Command the robot arm end-effector i.e. the gripper to close.
        """
        self.gripper.close(block=True)

    def is_left_arm(self):
        """
        Return true if the arm is corresponding to the left arm.
        """
        return self.__side_name == 'left'

    def is_right_arm(self):
        """
        Return true if the arm is corresponding to the right arm.
        """
        return self.__side_name == 'right'

class SceneObstacle:
    """
    A class responsible for representing an obstacle in the environment.
    A motion planning algorithm needs to avoid colliding with obstacles.
    """

    def __init__(self, obstacle_id, pos_x, pos_y, pos_z, obstacle_shape_dimensions=(0.0, 0.0, 0.0)):
        """
        Constructor.

        @param obstacle_id: represents a unique name for the obstacle.
        @param pos_x: represents the position or offset of the obstacle in the x-axis direction.
        @param pos_y: represents the position or offset of the obstacle in the y-axis direction.
        @param pos_y: represents the position or offset of the obstacle in the z-axis direction.
        @param obstacle_shape_dimensions: represent the shape of the object used for collision (length, width, depth or height).
        """

        self.name = obstacle_id
        self.pose = PoseStamped()
        self.pose.header.frame_id = None
        self.pose.header.stamp = rospy.Time.now()

        self.pose.pose.position.x = pos_x
        self.pose.pose.position.y = pos_y
        self.pose.pose.position.z = pos_z

        self.shape_size = obstacle_shape_dimensions

        # TODO: add obstacles orientation as well. get orientation as paramters as well.
        # q=quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))
        # self.pose.pose.orientation = Quaternion(*q)


    def set_obstacle_parent_frame_id(self, frame_id):
        """ Sets the parent frame of the obstacle so that we can represent the relative pose. """
        self.pose.header.frame_id = frame_id

    def __str__(self):
        """ Represent the scene obstacle in strings.
        """
        return '\n obstacle_id = {},\n position=({},{},{}),\n shape_size=({},{},{})\n'.format(self.name,
                                                                                     self.pose.pose.position.x,
                                                                                     self.pose.pose.position.y,
                                                                                     self.pose.pose.position.z,
                                                                                     self.shape_size[0],
                                                                                     self.shape_size[1],
                                                                                     self.shape_size[2])

class RobotEnvironmentFactory:
    """
    This class is used to implement factory design pattern.
    """

    __scene_environment_class = None

    @staticmethod
    def make_robot_scene_environment_factory(environment_type):
        """
        Abstract class constructs dynamic classes.

        paramters
        ---------
            type: string representing the type of environment to create.
        """
        if isinstance(environment_type, str):
            if environment_type == 'baxter_scene':
                rospy.loginfo('scene factory: creating baxter scene.')
                RobotEnvironmentFactory.__scene_environment_class = BaxterSimulationScene()
        else:
            raise TypeError('please specify the class type of the robot scene in strings.')

    @staticmethod
    def get_robot_scene_environment():
        """
        Return the required environment.
        """
        return RobotEnvironmentFactory.__scene_environment_class.clone()

class RobotSceneAbstractClass():
    """
    A class used to represent an abstract class for a robotic environment.
    """

    __metaclass__ = ABCMeta

    @abstractmethod
    def clone(self):
        """ This static method must be implemented in child classes. """
        pass

    @abstractmethod
    def get_obstacles(self):
        """ Return the environment list of obstacles. """
        pass

class BaxterSimulationScene(RobotSceneAbstractClass):
    """
    A class used to represent the custom 3D simulated environment for the
    vision-based object detection.
    """
    def __init__(self):
        """
        Constructor.

        """

        self.__obstacles = list()
        self.__create_scene_obstacles()

    def clone(self):
        """ This method is defined in the abstract class, thus needs implementation. """
        return copy.deepcopy(self)

    def get_obstacles(self):
        return self.__obstacles

    def __create_scene_obstacles(self):
        """
        This method dynamically creates our designed scene enviromnet obstacles.
        """

        file_path = rospkg.RosPack().get_path('vision_based_pick_and_place')+'/config/'
        config_path = file_path+'obstacles.json'

        file = open(config_path, 'r')
        data = json.load(file)

        if data is not None:

            for obstacle in data['obstacles']:
                position = map(float, str( obstacle['position'] ).split(' ') )
                shape = map(float, str( obstacle['shape'] ).split(' ') )

                obj = SceneObstacle(
                    obstacle_id=str(obstacle['obstacle_id']),
                    pos_x=position[0],
                    pos_y=position[1],
                    pos_z=position[2],
                    obstacle_shape_dimensions=(shape[0], shape[1], shape[2]))

                # Debugging !
                rospy.loginfo(str(obj))


                self.__obstacles.append(obj)

            BaxterSimulationScene.__obstacles = self.__obstacles
        else:
            rospy.logwarn('data from obstacles.json not loaded correctly')
            rospy.loginfo('prinint data {}'.format(data))

class RobotMotionPlanner:
    """
    This class is responsible for robot arm motion planning.
    Execute collision aware trajectories. Using MoveIt! framework.
    """

    def __init__(self):
        """
        Constructor.

        """

        # Initialize 'moveit_commander'.
        moveit_commander.roscpp_initialize(sys.argv)

        # RobotCommander object is an interface to the robot.
        self.baxter_robot = moveit_commander.RobotCommander()

        # Setup Baxter's arms.
        self.left_arm = RobotArm('left')
        self.right_arm = RobotArm('right')

        # Given a pose relative to the robot. We try to plan with both arms
        # to find a plan that successfully reach the desired pose
        # thus we need to keep track of the current active arm.
        self.current_active_arm = None


        self.__hover_distance = 0.05

        # Create the 'PlanningSceneInterface' object as an interface to the world
        # surrounding the robot used to add obstacles.

        self.scene = moveit_commander.PlanningSceneInterface()

        # Wait for RVIZ to visualize the obstacles.
        rospy.sleep(1.0)

        self.__build_scene()

        # To visualize trajectories in RVIZ we need to publish to a certain topic.
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=45)

        # baxter sdk inverse kinematics setup
        self.__name_space = None
        self.__iksvc = None

    def baxter_sdk_ik_service_setup(self, arm):
        """ Sets up Baxter SDK IK service.

        paramters
        ---------
        side: 'left' or 'right' representing baxter arm.
        """
        # Define Baxter SDK IK service

        self.current_active_arm = arm
        self.__name_space = 'ExternalTools/' + str(self.current_active_arm) + '/PositionKinematicsNode/IKService'
        self.__iksvc = rospy.ServiceProxy(self.__name_space, SolvePositionIK)
        rospy.wait_for_service(self.__name_space, 5.0)

    def baxter_sdk_move_to_start(self, initial_angles):
        """ given joint angles try to get baxter to move to.

        paramters
        ---------
        initial_angles: arm joint angles {s0, s1, e0, e1, w0, w1, w2}.
        """
        self.baxter_sdk_move_to_joint_positions(initial_angles)
        self.current_active_arm.open_active_arm_gripper()
        rospy.sleep(1.0)

    def baxter_sdk_ik_request(self, pose):
        """ Try to find a solution to the inverse kinematics of a given pose.

        paramters
        ---------
        pose: requested pose to find joint angles to reach it.
        """

        header = Header(stamp=rospy.Time.now(), frame_id='base')
        inverse_kinematics_request = SolvePositionIKRequest()
        inverse_kinematics_request.pose_stamp.append(PoseStamped(header=header, pose=pose))

        try:
            result = self.__iksvc(inverse_kinematics_request)
        except (ropsy.ServiceException, rospy.ROSException), e:
            rospy.logerr('service call failed: {}'.format(e))
            return False

        # Check if the result is valid. And type of seed used to get a solution
        # we convert rospy's string representation of uint8[]'s to int's
        result_seeds = struct.unpack('<%dB' % len(result.result_type), result.result_type)
        limb_joints = dict()

        if( result_seeds[0] != result.RESULT_INVALID ):
            seed_string = {
                inverse_kinematics_request.SEED_USER: 'User Provided Seed',
                inverse_kinematics_request.SEED_CURRENT: 'Current Joint Angles',
                inverse_kinematics_request.SEED_NS_MAP: 'Nullspace Setpoints',
            }.get(result_seeds[0], 'None')

            rospy.loginfo('IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}'.format((seed_string)))

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(result.joints[0].name, result.joints[0].position))

            rospy.loginfo('IK Solution: \n{0}'.format(limb_joints))
            rospy.loginfo('--------------')
        else:
            rospy.logerr('INVALID POSE - No Valid Joint Solution Found.')

        return limb_joints

    def baxter_sdk_move_to_joint_positions(self, joint_angles):
        """ Uses Baxter SDK position control to move to joint angles.

        paramters
        ---------
        joint_angles: the active current arm joint angles to set the manipulator joints to.
        """

        if joint_angles:
            self.current_active_arm.baxter_sdk_limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr('No Joint Angles provided for move_to_joint_positions')

    def baxter_sdk_approach(self, pose):
        """ Uses geometry pose representation to approach a desired
        pose with some hover distance.

        paramters
        ---------
        pose: desired pose to reach.
        """
        approach_pose = copy.deepcopy(pose)
        approach_pose.position.z = approach_pose.position.z + self.__hover_distance
        joint_angles = self.baxter_sdk_ik_request(approach_pose)
        self.baxter_sdk_move_to_joint_positions(joint_angles)

    def baxter_sdk_retract(self):
        """ After either grasp or release, the arm retracts to the approach pose.
        """

        current_pose = self.current_active_arm.baxter_sdk_limb.endpoint_pose()
        ik_pose = Pose()

        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z

        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        joint_angles = self.baxter_sdk_ik_request(ik_pose)

        self.baxter_sdk_move_to_joint_positions(joint_angles)

    def baxter_sdk_move_to_pose(self, pose):
        """ Move to the graping pose or releasing pose.

        paramters
        ---------
        pose: pose here could be either the pose to grasp or the pose to drop an object.
        """

        joint_angles = self.baxter_sdk_ik_request(pose)
        self.baxter_sdk_move_to_joint_positions(joint_angles)

    def baxter_sdk_pick(self, pose):
        """ Using Baxter SDK to pick an object.

        paramters
        ---------
        pose: the pose of the object to be reterieved.
        """

        # Open the gripper.
        self.open_active_arm_gripper()

        # Approach the desired pose.
        self.baxter_sdk_approach(pose)

        # Move to the pose.
        self.baxter_sdk_move_to_pose(pose)

        # Close the gripper.
        self.close_active_arm_gripper()

        # Retract
        self.baxter_sdk_retract()

    def baxter_sdk_place(self, pose):
        """ Using Baxter SDK to place an object.

        paramters
        ---------
        pose: the position and orientation of the object release.
        """

        # Appraoch the pose.
        self.baxter_sdk_approach(pose)

        # Move to the pose.
        self.baxter_sdk_move_to_pose(pose)

        # Open gripper.
        self.open_active_arm_gripper()

        # Retract to clear the object.
        self.baxter_sdk_retract()

    def __build_scene(self):
        """
            Construct a scene and add obstacles to MoveIt! world.

            This method make use of the factory design pattern to enable the creation
            of multiple environments as this enables the abstraction between actual
            implementation of classes and provide an interface than enable easy reuse
            and reduce code dependency.
        """

        # Prepare the scene as we initialize the factory of environments.
        RobotEnvironmentFactory.make_robot_scene_environment_factory('baxter_scene')

        # Get the designed robotic scene from the factory.The scene is a fully
        # contained environment with obstacles defined in the scene.
        robot_environment = RobotEnvironmentFactory.get_robot_scene_environment()

        # For each of the defined obstacles we add it to the scene in roder
        # to build collision aware algorithms.

        if robot_environment.get_obstacles() is not None:

            for obstacle in robot_environment.get_obstacles():
                obstacle.set_obstacle_parent_frame_id(self.baxter_robot.get_planning_frame())
                self.scene.add_box(obstacle.name, obstacle.pose, obstacle.shape_size)
        else:
            rospy.logwarn('robot environment has no obstacles added.')

    def move_to_given_pose(self, robot_endeffector_pose, arm):
        """ Let the robot arm move to the given desired pose. """
        self.current_active_arm = arm
        self.current_active_arm.moveit_limb.clear_pose_targets()
        self.current_active_arm.moveit_limb.set_pose_target(robot_endeffector_pose.get_pose())
        self.current_active_arm.moveit_limb.go(wait=True)

    def set_active_arm(self, arm):
        """ Sets the active arm for planning. Note. this may fail.
        """
        self.current_active_arm = arm
        self.current_active_arm.moveit_limb.clear_pose_targets()

    def is_pose_with_robot_reachable_workspace(self, pose):
        """
        Determine the Validity of given pose to robot base.
        To ensure that pose can be reached by the robot we need to define the workspace.
        """

        # upper_z, lower_z = 1.43, 0.91
        # upper_y, lower_y = 2.61, 0.00
        # upper_x, lower_x = 2.00, 0.00

        x = 200
        y = 261
        z = 143

        upper_x, lower_x = -x/2 * 0.01 , x/2 * 0.01
        upper_y, lower_y = -y/2 * 0.01 , y/2 * 0.01
        upper_z, lower_z = -z/2 * 0.01 , z/2 * 0.01

        if pose.position_x <= upper_x and pose.position_x >= lower_x:
            pass
        else:
            return False
        if pose.position_y <= upper_y and pose.position_y >= lower_y:
            pass
        else:
            return False
        if pose.position_z <= upper_z and pose.position_z >= lower_z:
            pass
        else:
            return False

        return True

    def open_active_arm_gripper(self):
        """ Will open the gripper of the current active arm. """
        if self.current_active_arm is not None:
            self.current_active_arm.open_gripper()
        else:
            rospy.logwarn('can\'t open gripper for nonetype arm')

    def close_active_arm_gripper(self):
        self.current_active_arm.close_gripper()

    def get_end_effector_current_pose(self, side_name):
        """
        Return the current pose of the end-effector.
        """
        arm = self.left_arm if side_name == 'left' else self.right_arm

        pose = arm.baxter_sdk_limb.endpoint_pose()

        position = pose['position']
        x, y, z = [ position.x, position.y, position.z ]

        rotation = pose['orientation']

        roll, pitch, yaw = 0, 1, 2
        euler = euler_from_quaternion(quaternion=(rotation.x, rotation.y, rotation.z, rotation.w))

        return RobotEndEffectorPose(x, y, z, euler[roll], euler[pitch], euler[yaw])

    def set_neutral_joint_position_of_limb(self):
        """ Move the robot to neutral positions. """
        left_arm_joints = {
            'left_s0': 0.0,
            'left_s1': -0.55,
            'left_e0': 0.0,
            'left_e1': 0.75,
            'left_w0': 0.0,
            'left_w1': 1.26,
            'left_w2': 0.0
        }

        right_arm_joints = {
            'right_s0': 0.0,
            'right_s1': -0.55,
            'right_e0': 0.0,
            'right_e1': 0.75,
            'right_w0': 0.0,
            'right_w1': 1.26,
            'right_w2': 0.0
        }

        if self.current_active_arm is not None:

            if self.current_active_arm.is_left_arm():
                self.current_active_arm.moveit_limb.set_joint_value_target(left_arm_joints)
                self.current_active_arm.moveit_limb.go(wait=True)
            elif self.current_active_arm.is_right_arm():
                self.current_active_arm.moveit_limb.set_joint_value_target(right_arm_joints)
                self.current_active_arm.moveit_limb.go(wait=True)

        else:
            rospy.logwarn('set_neutral_joint_position_of_limb(): no current active arm is enabled.')

    def move_to_start_pose(self, start_angles=None):
        """ Move to a start configuration state.
        """
        if self.current_active_arm is not None:
            self.current_active_arm.moveit_limb.set_joint_value_target(start_angles)
            self.current_active_arm.moveit_limb.go(wait=True)
        else:
            rospy.logwarn('move_to_start_pose(): not current active arm is enabled.')

class PickNPlaceDemo(object):
    """
    This class is used to implement an object manipulation task of a
    pick and place based on 3D perception.
    """

    def __init__(self):
        """
        Constructor.

        """

        # First we need to enable the robot.
        self.robot_state = baxter_interface.RobotEnable( CHECK_VERSION )
        self.initial_state = self.enable_robot()
        rospy.on_shutdown(self.disable_robot)

        # Distance to move aware from the pose as approaching
        self.__hover_distance = 0.15

        self.planner = RobotMotionPlanner()
        # self.baxter_sdk_move_to_start()

        self.move_to_start()
        # if self.planner.is_pose_with_robot_reachable_workspace(RobotEndEffectorPose(0.7, 2.7, 1.0, 0.0, 0.0, 0.0)):
        #     self.pick_object(0.7, 2.7, 1.0, 0.0, 0.0, 0.0)

        # self.planner.set_active_arm(self.planner.left_arm)
        # self.planner.set_neutral_joint_position_of_limb()

    def baxter_sdk_move_to_start(self):

        left_arm_start_joint_angles = {
            'left_s0': -0.7648,
            'left_s1': -0.7653,
            'left_e0':  0.0000,
            'left_e1':  1.1041,
            'left_w0':  0.0000,
            'left_w1':  0.9411,
            'left_w2':  1.3405
        }

        right_arm_start_joint_angles = {
            'right_s0':  0.7648,
            'right_s1': -0.7653,
            'right_e0':  0.0000,
            'right_e1':  1.1041,
            'right_w0':  0.0000,
            'right_w1':  0.9411,
            'right_w2':  1.3405
        }

        # self.planner.baxter_sdk_ik_service_setup(self.planner.left_arm)
        # self.planner.baxter_sdk_move_to_joint_positions(left_arm_start_joint_angles)

        self.planner.baxter_sdk_ik_service_setup(self.planner.right_arm)
        self.planner.baxter_sdk_move_to_joint_positions(right_arm_start_joint_angles)

        overhead_orientation = Quaternion( x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
        self.planner.baxter_sdk_pick(Pose(position=Point(0.7, 2.7, 1.0), orientation=overhead_orientation))

    def enable_robot(self):
        """ Make sure robot is ready for control commands.
        """
        state = False
        if not self.robot_state.state().enabled:
            rospy.loginfo('Enabling robot.')
            self.robot_state.enable()
            state = True
        return state

    def disable_robot(self):
        """ Make sure robot is correctly shutdown before exiting.
        """
        state = False
        if self.robot_state.state().enabled:
            rospy.loginfo('Disabling robot.')
            self.robot_state.disable()
            state = True
        return state

    def __approach(self, end_effector_pose):
        """ Approach rear the object with a hover distance.
        """
        approach = copy.deepcopy(end_effector_pose)
        approach.position_z = approach.position_z + self.__hover_distance
        self.planner.move_to_given_pose(approach, self.planner.current_active_arm)

    def __retract(self):
        """ After pick or place operation we retract to the initial approach hover distance.
        """
        pose = self.planner.get_end_effector_current_pose(self.planner.current_active_arm)
        pose.position_z = pose.position_z + self.__hover_distance
        self.planner.move_to_given_pose(pose, self.planner.current_active_arm)

    def __move_to_grasp_or_release_pose(self, grasp_pose):
        """ After we approach, we move to the graspable pose for pick or place.
        """
        object_pose = copy.deepcopy(grasp_pose)
        self.planner.move_to_given_pose(object_pose, self.planner.current_active_arm)

    def pick_object(self, object_pick_pose):
        """ Implement a pick (grasp) sequence.
        """

        # Open the gripper
        self.planner.open_active_arm_gripper()

        # Approach the object with a hover distance.
        self.__approach(object_pick_pose)

        # Move to the graspable pose
        self.__move_to_grasp_or_release_pose(object_pick_pose)

        # Close the gripper
        self.planner.close_active_arm_gripper()

        # Retract to clear object
        self.__retract()

    def place_object(self, object_place_pose):
        """ Implement a place (release) sequence.
        """

        # Move to place pose
        self.__approach(object_place_pose)

        # Go the drop or release
        self.__move_to_grasp_or_release_pose(object_place_pose)

        # Open the grippers
        self.planner.open_active_arm_gripper()

        # Retract to clear object
        self.__retract()

    def move_to_start(self):
        # Define start pose angles
        # Object Grasp pose
        #    0.8 -0.2 0.88
        # 0.899869, -0.199603 0.892955
        # Object Release Pose
        #   0.10 -0.2 0.88
        # Call pick and place

        left_arm_start_joint_angles = {
            'left_s0': -0.7648,
            'left_s1': -0.7653,
            'left_e0':  0.0000,
            'left_e1':  1.1041,
            'left_w0':  0.0000,
            'left_w1':  0.9411,
            'left_w2':  1.3405
        }

        left = {'left_s0':-0.9751,'left_s1':-0.6397,'left_e0':-0.6397, 'left_e1':0.5151, 'left_w0':0.0, 'left_w1':0.8381, 'left_w2':1.5811}
        left1 = {'left_s0':-0.7265,'left_s1':-0.7473,'left_e0':0.0, 'left_e1':0.0, 'left_w0':0.0, 'left_w1':0.0, 'left_w2':0.0}

        right_arm_start_joint_angles = {
            'right_s0':  0.7648,
            'right_s1': -0.7653,
            'right_e0':  0.0000,
            'right_e1':  1.1041,
            'right_w0':  0.0000,
            'right_w1':  0.9411,
            'right_w2':  1.3405
        }

        self.planner.set_active_arm(self.planner.left_arm)
        self.planner.move_to_start_pose(left1)

        # self.planner.set_active_arm(self.planner.right_arm)
        # self.planner.move_to_start_pose(right_arm_start_joint_angles)

class Test:
    def __init__(self):
       PickNPlaceDemo()

if __name__ == "__main__":
    rospy.init_node('node', anonymous=True)
    try:
        Test()
        rospy.spin()
    except:
        pass

    moveit_commander.os._exit(0)































