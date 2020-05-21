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

import rospy
import tf


import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion

class RGBCameraTransform:

    """
        Publish the relative pose of the camera in the scene.
        Useful with using with RVIZ visualizer as it can help visualize
        the relative poses of objects in the scene relative to the camera correctly.
    """

    def __init__(self, relative_pose = "None"):

        rospy.init_node('camera_transform', anonymous=True)

        broadcaster = tf2_ros.StaticTransformBroadcaster()

        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = 'world'
        static_transformStamped.child_frame_id  = 'openni_camera_link'

        if relative_pose != None:
            rospy.loginfo('about to publish camera static transform')
            static_transformStamped.transform.translation.x = relative_pose.position.x
            static_transformStamped.transform.translation.y = relative_pose.position.y
            static_transformStamped.transform.translation.z = relative_pose.position.z
            quaternion = tf.transformations.quaternion_from_euler(
                float(relative_pose.orientation.x),
                float(relative_pose.orientation.y),
                float(relative_pose.orientation.z)
                )
            static_transformStamped.transform.rotation.x = quaternion[0]
            static_transformStamped.transform.rotation.y = quaternion[1]
            static_transformStamped.transform.rotation.z = quaternion[2]
            static_transformStamped.transform.rotation.w = quaternion[3]
            broadcaster.sendTransform(static_transformStamped)
        else:
            static_transformStamped.transform.translation.x = 1.7
            static_transformStamped.transform.translation.y = 0.0
            static_transformStamped.transform.translation.z = 1.508
            quaternion = tf.transformations.quaternion_from_euler(float(0.0), float(0.0), float(-2.43778))
            static_transformStamped.transform.rotation.x = quaternion[0]
            static_transformStamped.transform.rotation.y = quaternion[1]
            static_transformStamped.transform.rotation.z = quaternion[2]
            static_transformStamped.transform.rotation.w = quaternion[3]
            broadcaster.sendTransform(static_transformStamped)


if __name__ == '__main__':
    try:
        camera_relative_pose = Pose(position=Point(x=1.7, y=0.0, z=1.508), orientation=Quaternion(x=0.0,y=0.0,z=-2.43778, w=1.0))
        RGBCameraTransform(relative_pose=camera_relative_pose)
        rospy.spin()
    except KeyboardInterrupt as e:
        print(e)
