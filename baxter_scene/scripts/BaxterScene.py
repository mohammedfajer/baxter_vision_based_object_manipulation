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

import struct
import sys
import copy

import rospy
import rospkg

import roslib
roslib.load_manifest("baxter_scene")

import os


from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

import json

class JsonObject(object):

    """
        JsonObject Class used to work with json files. Used to create
        dynamic custom Gazebo environments.
    """

    def __init__(self, path):
        self.__dict__ = self.ReadJsonFile(path)

    def ReadJsonFile(self, path):
        data=open(path, 'r').read()
        try:
            return (json.loads(data))
        except ValueError, e:
            raise MalformedJsonFileError('{} when reading {}'.format(str(e),path))

    def PrintData(self):
        print(self.__dict__)

    def GetDictionary(self):
        return self.__dict__

    def GetArrayJson(self):
        models=list()
        for model in self.__dict__['models']:
            model_info = {'name':None, 'path':None, 'pose':None, 'reference_frame':None}
            model_info['name']=str(model['name'])
            model_info['path']=str(model['path'])
            model_info['pose']=model['pose']
            model_info['reference_frame']=str(model['reference_frame'])
            models.append(model_info)
        return models

class BaxterScene(object):
    """
        BaxterScene Class is responsible for loading and building a custom gazebo world.
        Simply by loading the model xml and specifying its pose relative to
        a specified reference Frame.
    """
    def ReadXML(self, path):
        """
            This method loads the Simulation Description Format (SDF) of a model
            to be rendered in Gasebo.
        """
        xml=''
        with open(path, 'r') as file:
            xml=file.read().replace('\n', '')
        return xml

    def Spawn(self, path, name, pose, reference_frame):
        """
            This method spawn a given model in Gazebo Simulation.
        """
        xml=self.ReadXML(path)
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf1 = spawn_sdf(name, xml, "/", pose, reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    def LoadGazeboModels(self):
        """
            This method make use of the above methods to load and build the Simulation environment.
        """
        middle_table_pose      = Pose(position=Point(1,0,0))
        left_table_pose        = Pose(position=Point(0,1.1,0))
        right_table_pose       = Pose(position=Point(0,-1.1,0))
        banana_pose            = Pose(position=Point(0.8,0.2,0.74+0.06))
        cheezeit_pose          = Pose(position=Point(0.8,-0.2,0.74+0.04+0.04+0.04+0.02))
        left_box_pose          = Pose(position=Point(0,1.1,0.74+0.04))
        right_box_pose         = Pose(position=Point(0,-1.1,0.74+0.04))
        camera_stand_pose      = Pose(position=Point(1.7,0,0), orientation=Quaternion(0,0,3.75246,1))
        left_lamp_pose         = Pose(position=Point(-1,2,0.91))
        right_lamp_pose        = Pose(position=Point(-1,-2,0.91))
        floor_pose             = Pose(position=Point(0.3,0,0.00625))
        wall_pose              = Pose(position=Point(-4,-4.1,0))
        side_wall_pose_left    = Pose(position=Point(-4,-4.1,0), orientation=Quaternion(0,0,-1.01229,1))
        side_wall_pose_right   = Pose(position=Point(4.7,3.5,0), orientation=Quaternion(0,0,0.95993129251877,1))
        rgb_d_sensor_pose      = Pose(position=Point(1.7,0,1.4), orientation=Quaternion(a))
        world_reference_frame  = 'world'

        # Get Models' Path
        path = rospkg.RosPack().get_path('baxter_scene')+'/models/'
        print(path)

        self.Spawn(path+"PnPTable/model.sdf"    , "MiddleTable" , middle_table_pose , world_reference_frame)
        self.Spawn(path+"PnPTable/model.sdf"    , "LeftTable"   , left_table_pose   , world_reference_frame)
        self.Spawn(path+"PnPTable/model.sdf"    , "RrightTable" , right_table_pose  , world_reference_frame)

        self.Spawn(path+"PnPBox/model.sdf"      , "Left Box"    , left_box_pose     , world_reference_frame)
        self.Spawn(path+"PnPBox/model.sdf"      , "Right Box"   , right_box_pose    , world_reference_frame)

        self.Spawn(path+"PnPBanana/model.sdf"   , "Banana"      , banana_pose       , world_reference_frame)
        self.Spawn(path+"PnPCheezeIt/model.sdf" , "CheezeIt"    , cheezeit_pose     , world_reference_frame)

        self.Spawn(path+"camera_stand/model.sdf", "Camera Stand", camera_stand_pose , world_reference_frame)
        self.Spawn(path+"lamp/model.sdf"        , "Left Lamp"   , left_lamp_pose    , world_reference_frame)
        self.Spawn(path+"lamp/model.sdf"        , "Right Lamp"  , right_lamp_pose   , world_reference_frame)

        self.Spawn(path+"floor/model.sdf"       , "Floor"       , floor_pose        , world_reference_frame)
        self.Spawn(path+"wall_one/model.sdf"    , "Back Wall"   , wall_pose         , world_reference_frame)
        self.Spawn(path+"wall/model.sdf"        , "Left Side-Wall", side_wall_pose_left, world_reference_frame)
        self.Spawn(path+"wall/model.sdf"        , "Right Side-Wall", side_wall_pose_right, world_reference_frame)

        self.Spawn(path+"kinect/model.sdf"      , "RGB-D Kinect Sensor", rgb_d_sensor_pose, world_reference_frame)

    def LoadCustomWorldJson(self):
        """
            This method implements a dynamic scene construction using a custom
            world json file.
        """

        jsonPath = rospkg.RosPack().get_path('baxter_scene')+'/custom_worlds/world.json'
        modelsPath = rospkg.RosPack().get_path('baxter_scene')+'/models/'

        data=JsonObject(jsonPath)

        models=data.GetArrayJson()

        for model in models:

            position_list =  map(float, str( model['pose']['position'] ).split(' ') )
            orientation_list =  map(float, str( model['pose']['orientation'] ).split(' ') )

            position    = Point(position_list[0], position_list[1], position_list[2])
            orientation =  Quaternion(float(orientation_list[0]), float(orientation_list[1]), float(orientation_list[2]), float(orientation_list[3]))

            pose = Pose(position=position, orientation=orientation)

            object_path = modelsPath + model['path']

            self.Spawn(object_path,model['name'],pose,model['reference_frame'])

def main():
    rospy.init_node('baxter_scene', anonymous=True)
    scene = BaxterScene()
    scene.LoadCustomWorldJson()
    rospy.spin()
if __name__ == "__main__":
    main()
