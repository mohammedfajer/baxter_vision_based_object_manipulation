#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import baxter_interface
from baxter_interface import CHECK_VERSION
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header
from moveit_msgs.msg import RobotState

import tf

def compute_FK(group, joint_values, links):
    rospy.wait_for_service('compute_fk')
    try:
        compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = group.get_pose_reference_frame()

        robot_state = RobotState()
        robot_state.joint_state.header = header
        robot_state.joint_state.name = group.get_active_joints()
        robot_state.joint_state.position = joint_values

        result = compute_fk( header, links, robot_state )

        return result.pose_stamped
    except rospy.ServiceException, e:
        rospy.INFO( 'service call failed: %s' % e )


def run():
    print('========== starting robot_arm_motion.py ==========')
    rospy.init_node('robot_arm_motion', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    # RobotCommander object: an interface to the robot as a whole
    baxter_robot = moveit_commander.RobotCommander()

    # PlanningSceneInterface object: an interace to the world surrounding the robot
    scene = moveit_commander.PlanningSceneInterface()

    # MoveGroupCommander object: an interface to one group of joints
    group = moveit_commander.MoveGroupCommander('left_arm')
    group.allow_replanning(True)
    group.set_pose_reference_frame('base')
    group.set_planner_id('RRTConnectKConfigDefault')
    group.set_goal_tolerance(0.1)
    group.set_num_planning_attempts(10)
    group.set_planning_time(5.0)
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)
    
    # DisplayTrajectory publisher: publish trajectories for RVIZ to visualize
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

    print('========== waiting for RVIZ ==========')
    rospy.sleep(10)

    print('========== baxter reference frame: %s' % group.get_planning_frame())
    print('========== baxter end effector link: %s' % group.get_end_effector_link())
    print('========== baxter joint groups: %s' % baxter_robot.get_group_names())
    print('========== baxter robot state: %s' % baxter_robot.get_current_state())

    print('========== generating plan 1 ==========')
    # left_arm_initial_joint_space = {'left_s0': -0.8030, 'left_s1': -0.6756, 'left_e0': 0.0000, 'left_e1': 1.8086, 'left_w0': 0.0000, 'left_w1': 0.0000, 'left_w2': 0.0000}
    # joint_variables=[left_arm_initial_joint_space[i] for i in left_arm_initial_joint_space]
    # pose=compute_FK(group, joint_variables, ['left_hand'] )
    # group.set_pose_target(pose[0].pose)
    # plan1=group.go()

    pose_goal=geometry_msgs.msg.Pose()
    pose_goal.orientation.w=1.0
    pose_goal.position.x=0.4
    pose_goal.position.y=0.1
    pose_goal.position.z=0.4
    group.set_pose_target(pose_goal)
    plan=group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


    # target_pose = geometry_msgs.msg.Pose()
    # target_pose.orientation.w = 1.0
    # target_pose.position = geometry_msgs.msg.Point(0.7, -0.05, 1.1)
    # group.set_pose_target(target_pose)

    # plan1 = group.plan()
    # group.execute(plan1,  wait=True)

    print('========== waiting while RVIZ displays plan 1 ==========')
    rospy.sleep(5)

    # moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
