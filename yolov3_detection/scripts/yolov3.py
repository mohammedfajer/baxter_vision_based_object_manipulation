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

import yaml
import json

import cv2
import numpy as np
import random
import copy

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os


class YOLOv3(object):

    """
        This class uses 'You Only Look Once (YOLO)' version 3 State of the art,
        real-time object detection system. It is a convolutional
        neural network (CNN) aimed for object detection.

        This class is based on a custom object detection neural network.
        The classes are 'CheezeIt' snack box and 'Banana'.
    """

    def __init__(self):
        """
            The constructor responsible for initializing the network and
            prepare it for running.
        """

        # Initialize network paramters
        self.confidance_threshold = None
        self.non_maximum_suppression_threshold = None
        self.network_input_image_width = None
        self.network_input_image_height = None

        # Network configurations
        self.classes_file = None
        self.model_configuration = None
        self.model_weights = None
        self.classes = list()
        self.network = None

        self.IsnetworkLoaded = False
        self.IsnetworkParamLoaded = False

        self.LoadNetworkParamters()
        self.LoadNetwork()


    def LoadNetworkParamters(self):
        """
            This method loads the network initial paramters that are
            stored in 'network_paramters.yaml' file.
        """

        if self.IsnetworkParamLoaded == False:


            network_paramters_file = open('src/baxter_perception_and_control/baxter_perception/yolov3_detection/network_paramters.yaml', 'r')
            paramters_dictionary = yaml.load(network_paramters_file)

            self.confidance_threshold = paramters_dictionary['confThreshold']
            self.non_maximum_suppression_threshold = paramters_dictionary['nmsThreshold']
            self.network_input_image_width = paramters_dictionary['inputWidth']
            self.network_input_image_height = paramters_dictionary['inputHeight']

            self.IsnetworkParamLoaded = True


            rospy.loginfo( "Confidence Threshold : {} ".format(self.confidance_threshold) )
            rospy.loginfo( "Non-Maximum Suppression Threshold : {} ".format(self.non_maximum_suppression_threshold) )
            rospy.loginfo( "Network Input Image Width : {} ".format( self.network_input_image_width ) )
            rospy.loginfo( "Network Input Image Height : {} ".format( self.network_input_image_height ) )
            rospy.loginfo( "IsnetworkParamLoaded : {} ".format(self.IsnetworkParamLoaded))

    def LoadNetwork(self):
        """
            This method loads the network weights and configuration file as well
            as setup some network settings.
        """

        if self.IsnetworkLoaded == False:


            path = 'src/baxter_perception_and_control/baxter_perception/yolov3_detection/network_configuration.json'
            data = open(path, 'r').read()
            try:
                network_config_paths = (json.loads(data))
            except ValueError, e:
                raise MalformedJsonFileError('{} when reading {}'.format(str(e), path))

            # Load names of the classes
            self.classes_file = network_config_paths['object_names']
            with open(self.classes_file, 'rt') as file:
                self.classes = file.read().rstrip('\n').split('\n')

            #  Setup the network
            self.model_weights = network_config_paths['pre_trained_weights_path']
            self.model_configuration = network_config_paths['network_configuration_path']

            self.network = cv2.dnn.readNetFromDarknet(self.model_configuration, self.model_weights)
            self.network.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.network.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

            self.IsnetworkLoaded = True

            rospy.loginfo( "pre_trained_weights_path : {} ".format(self.model_weights) )
            rospy.loginfo( "network_configuration_path : {} ".format(self.model_configuration) )
            rospy.loginfo( "IsnetworkLoaded : {} ".format( self.IsnetworkLoaded ) )



    def RunNetwork(self, cv_image):
        """
            This method runs the network on a given opencv image captured
            from the simulated RGB-D Xtion or Kinect Sensor in Gazebo.

            Return:
                Image window showing the detection.
        """

        # cv2.imshow("R", cv_image)
        # cv2.waitKey(3)

        self.LoadNetworkParamters()
        self.LoadNetwork()


        layer_names = self.network.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in self.network.getUnconnectedOutLayers()]
        colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        # image = cv2.resize(cv_image, None, fx=0.4, fy=0.4)
        image=cv_image

        image_height, image_width, channel_layers = image.shape

        # Detecting objects
        blob = cv2.dnn.blobFromImage(image, 0.00392,(image_width, image_height), (0, 0, 0), True, crop=False)

        self.network.setInput(blob)
        outs = self.network.forward(output_layers)

        # Information to be printed
        classes_ids = list()
        confidences = list()
        boxes = list()

        for out in outs:

            for detection in out:

                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > self.confidance_threshold:

                    # Object detected

                    print(class_id)

                    center_x = int(detection[0] * image_width)
                    center_y = int(detection[1] * image_height)

                    width = int(detection[2] * image_width)
                    height = int(detection[3] * image_height)

                    # Rectangle coordinates

                    x = int(center_x - width / 2)
                    y = int(center_y - height / 2)

                    boxes.append([x, y, width, height])
                    confidences.append(float(confidence))
                    classes_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.confidance_threshold, self.non_maximum_suppression_threshold)

        print(indexes)

        font = cv2.FONT_HERSHEY_PLAIN

        for i in range(len(boxes)):

            for i in indexes:

                x, y, w, h = boxes[i]
                label = str(self.classes[classes_ids[i]])
                color = colors[classes_ids[i]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                cv2.putText(image, label, (x - 50, y - 30), font, 3, color, 2)

        cv2.imshow("Camera Feed", image)
        key = cv2.waitKey(0)

        cv2.destroyAllWindows()


class ObjectDetection:

    """
        This class is responsible for gathering the converted RGBD image from ros to opencv
        and using it to run the YOLOv3 on it to detect the objects.
    """

    def __init__(self):
        rospy.init_node('object_detection', anonymous=True)
        self.image = None
        self.bridge = CvBridge()
        rospy.Subscriber('/opencv_bridge/output_video', Image, self.callback)

    def callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # cv2.imshow("R", self.image)
        # cv2.waitKey(3)

        YOLOv3().RunNetwork(copy.deepcopy(self.image))


if __name__ == '__main__':

    try:
        ObjectDetection()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
