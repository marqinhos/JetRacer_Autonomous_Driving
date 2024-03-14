#!/usr/bin/env python3

#
# This file is part of the repo: https://github.com/marqinhos/JetRacer_Autonomous_Driving
# If you find the code useful, please cite the Author: Marcos Fernandez Gonzalez
# 
# Copyright 2023 The JetRacer Autonomous Driving Author. All Rights Reserved.
#
# Licensed under the AGPL-3.0 License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/agpl-3.0.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ========================================================================================


import rospy
import cv2
import numpy as np
import os 
import threading as th
import signal
import yaml

# Import YOLO
from ultralytics import YOLO

# Import type of msgs
from sensor_msgs.msg import Image
from jetracer_speedway_msgs.msg import Dictionary, KeyValue, Points
# Import converter ros image to cv2 image
from cv_bridge import CvBridge

# Import utils
from utils import Point, ObjectProcessing


class ObjectDetection(th.Thread):
    """Class to subscribe a topic to take image and by means of IA extract the lane and lines segmentation.
        Them process the segmentation, to extract the desired point to go the robot
    """

    def __init__(self, model_ia_path: str) -> "ObjectDetection":

        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)
        
        ########################### YAML ###########################
        # Load config.yaml
        yaml_path = rospy.get_param('config_file')
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        ########################### Running ###########################
        self.running = True

        ########################### IA ###########################
        self.model_ia_path = model_ia_path
        self.model_ia = YOLO(self.model_ia_path)
        ## Information to select the device to run the model
        ## https://docs.ultralytics.com/modes/predict/#streaming-source-for-loop
        ## self.model_ia.to('cuda')
    
        self.predict_ia_conf = config["confidence"]

        ########################### IMAGE ###########################
        self.frame = None
        self.bridge = CvBridge()

        #################### Object Processing ####################
        self.object = ObjectProcessing()

        ########################### ROS ###########################
        ## Constants
        self.name_sub = config["sensors"]["pub_name_img"]
        self.name_pub = config["navigation"]["pub_name_obj"]        
        self.name_ros_node = config["navigation"]["node_name_obj"]  

        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)
        ## Publisher and Subscribers
        self.pub_dict = rospy.Publisher(self.name_pub, Dictionary, queue_size=1) ## TODO
        rospy.wait_for_message(self.name_sub, Image)
        self.sub_img = rospy.Subscriber(self.name_sub, Image, self.__callback_image)
        
       

    def run(self) -> None:
        """Main void
        """
        rospy.loginfo("Processing image...")
        while self.running:
            ## Wait for a frame
            if self.frame is None:
                self.rospyRate.sleep()
                continue
            
            ## Execute ia to extract features
            result = self.__run_ia(self.frame)

            dict_objects = self.object.run(result)

            ## Publish dictionary
            self.publish_command(dict_objects)

            ## Show Point in image
            ##self.__show(self.frame, dict_objects)

            ## Show segmentation using Yolov8 method
            ##self.__show_segmentation(result)

            ## Publish desired point
            # self.publish_point(desired_pt)
            # rospy.loginfo(f"Point: {desired_pt}")
            self.rospyRate.sleep()

        rospy.loginfo(f"Shutdown {self.name_ros_node}")


    def publish_command(self, result_objects: dict) -> None:
        """Void to publish a dictionary, with:
                - Key: name of object detection (see object_process.py to now possible detections)
                - Value: for each key is a list with centroid of all object detections.
            Only publish car and stop sign info. TODO for the rest of detections.

        Args:
            result_objects (dict): Dictionary with all objects detections
        """
        ## Only publish car and stop info

        ## Create obj command
        dict_msg = Dictionary()
        
        ## Check for objects
        try:
            for name in list(result_objects.keys()):
                name_detect = self.object.dict_keys_detect_swap[name]
                key_value = KeyValue()
                key_value.key = name_detect
                for point in result_objects[name]:
                    point_cc = Points()
                    point_cc.x = point.x
                    point_cc.y = point.y
                    key_value.value.append(point_cc)

                dict_msg.dict.append(key_value)
                    
            self.pub_dict.publish(dict_msg)
        except: 
            self.pub_dict.publish(dict_msg)
            

    def __callback_image(self, data: Image) -> None:
        """Callback to set in self variable the values of image. Convert the type of data to opencv image

        Args:
            data (Image): Image in format image msg 
        """
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.frame = image


    def __show(self, image: np.ndarray, result: dict) -> None:
        """Void to show desired point, and real point 

        Args:
            image (np.ndarray): Image to show
            result (dict): Dictionary with all objects detections
        """
        try:
            for name in list(result.keys()):
                for point in result[name]:
                    cv2.circle(image, (point.x, point.y), 5, (0, 0, 255), -1)
        except: pass

        cv2.imshow(self.name_ros_node , image)
        cv2.waitKey(3)


    def __show_segmentation(self, results: list) -> None: 
        """Void to show detections using new YOLOv8 method. 

        Args:
            results (list): Dictionary with all objects detections
        """
        for result in results:
            ## Create image with bounding boxes
            annotated_frame = result.plot()
            ## Show
            cv2.imshow("Segementation of detected areas", annotated_frame)
            cv2.waitKey(3)



    def __run_ia(self, frame: np.ndarray) -> list:
        """Function to segment a frame with own IA

        Args:
            frame (np.ndarray): Frame to predict

        Returns:
            list: List of result (masks, boxes, image) that return prediction in model YOLOv8
        """
        ## Run model in gpu or cpu with flag
        return self.model_ia.predict(source=frame, conf=self.predict_ia_conf, device='cpu')
    

def signal_handler(signal, frame) -> None:
    rospy.logwarn("Ctrl+C detected, stopping object thread...")
    object_detection.running = False

        
if __name__ == '__main__':
    try:
        ## Absolute path
        file_path = os.path.abspath(__file__)
        ## Absolute path of folder models
        models_dir = os.path.join(os.path.dirname(file_path), 'models')
        ## Model IA path
        model_path = os.path.join(models_dir, 'yolov8n-seg.pt')
        ## Call Process Image class
        object_detection = ObjectDetection(model_ia_path=model_path)
        ## Manage SIGINT signal
        signal.signal(signal.SIGINT, signal_handler)
        ## Upload thread
        list_threads = [object_detection]
        ## Start and Join threads
        [wire.start() for wire in list_threads]
        [wire.join() for wire in list_threads]

    except rospy.ROSInterruptException:
        rospy.loginfo("Close this program")
        pass
