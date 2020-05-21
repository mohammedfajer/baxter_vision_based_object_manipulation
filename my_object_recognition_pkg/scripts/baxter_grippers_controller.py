#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg 
import baxter_interface
from baxter_interface import CHECK_VERSION

def run():
    rospy.init_node('baxter_gripper_controller', anonymous=True)
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    leftgripper = baxter_interface.Gripper('left', CHECK_VERSION)
    rightgripper = baxter_interface.Gripper('right', CHECK_VERSION)

    leftgripper.close()
    rospy.sleep(2.0)
    leftgripper.open()
    rospy.sleep(2.0)
    
    # rightgripper.close()
    # rospy.sleep(1.0)

if __name__ == "__main__":
    try:
        run()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass