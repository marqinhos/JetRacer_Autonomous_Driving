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
from geometry_msgs.msg import Twist
from jetracer_speedway_msgs.msg import Points
# Import converter ros image to cv2 image
from cv_bridge import CvBridge

# Import utils
from utils import Point, Features_Detection
# Import detections features
# from .utils.image_process import Features_Detection



class ProcessImage(th.Thread):
    """Class to subscribe a topic to take image and by means of IA extract the lane and lines segmentation.
        Them process the segmentation, to extract the desired point to go the robot
    """

    def __init__(self, model_ia_path: str) -> "ProcessImage":

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
        self.predict_ia_conf = config["confidence"]

        ########################### IMAGE ###########################
        self.frame = None
        self.bridge = CvBridge()
        self.last_message_seq = None

        #################### FEATURES DETECTION ####################
        self.compute_detect = Features_Detection()
        self.last_desired_pt = Point(0, 0)

        ########################### ROS ###########################
        ## Constants
        self.name_sub = config["sensors"]["pub_name_img"]
        self.name_pub = config["navigation"]["pub_name_img"]
        self.name_ros_node = config["navigation"]["node_name_img"]
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)
        ## Publisher and Subscribers
        self.pub_vels = rospy.Publisher(self.name_pub, Points, queue_size=1)
        rospy.wait_for_message(self.name_sub, Image)
        self.sub_img = rospy.Subscriber(self.name_sub, Image, self.__callback_image)
        
       

    def run(self) -> None:
        """Main void
        """

        while self.running:
            ## Wait for a frame
            if self.frame is None:
                self.rospyRate.sleep()
                continue
            
            ## Execute ia to extract features
            result = self.__run_ia(self.frame)

            ## Get desired Point
            try:
                #Cuando la IA detecta algo, esta funcion devuelve None
                desired_pt = self.compute_detect.run(result)
                if not self.last_desired_pt.zero():
                    if abs(self.last_desired_pt.x - desired_pt.x) > 95 or abs(self.last_desired_pt.y - desired_pt.y) > 80:
                        rospy.logwarn("BAD DESIRED POINT")
                        desired_pt = self.last_desired_pt.middle_2_point(desired_pt)
                self.last_desired_pt = desired_pt
            except:
                desired_pt = self.last_desired_pt

            ## Show Point in image
            self.__show(self.frame, desired_pt)

            ## Show segmentation using Yolov8 method
            ##self.__show_segmentation(result)

            ## Publish desired point
            self.publish_point(desired_pt)
            # rospy.loginfo(f"Point: {desired_pt}")
            self.rospyRate.sleep()


        rospy.loginfo(f"Shutdown {self.name_ros_node}")
        
    
    def publish_point(self, desired_point: Point) -> None:
        """Void to publish desired point in Points format. The topic to publish is "jetracer_vels"

        Args:
            angle_deg (float): Angle
        """

        real_pose = Point(self.compute_detect.size[0]//2, self.compute_detect.size[1]-10)
        # Create Points message
        point_msg = Points()
        point_msg.x = desired_point.x
        point_msg.y = desired_point.y
        point_msg.xr = real_pose.x
        point_msg.yr = real_pose.y
        
        # Publish message in topic vels_jetracer
        self.pub_vels.publish(point_msg)


    def __show(self, image: np.ndarray, point: Point) -> None:
        """Void to show desired point, and real point 

        Args:
            image (np.ndarray): Image to show
            point (Point): Point to draw in image
        """

        cv2.circle(image, (point.x, point.y), 5, (0, 0, 255), -1)
        cv2.circle(image, (self.compute_detect.size[0]//2, self.compute_detect.size[1]-10), 5, (255, 0, 0), -1)
        cv2.imshow(self.name_ros_node , image)
        cv2.waitKey(3)


    def __show_segmentation(self, results: list) -> None: 
        """Void to show detections using new YOLOv8 method. 

        Args:
            results (list): Dictionary with all objects detections
        """
        for result in results:
            result.show()  # display to screen

    
    def __callback_image(self, data: Image) -> None:
        """Callback to set in self variable the values of image. Convert the type of data to opencv image

        Args:
            data (Image): Image in format image msg 
        """

        current_time = rospy.Time.now()
        msg_time = data.header.stamp

        time_diff = current_time - msg_time
        rospy.loginfo(f"Delay de la red: {time_diff.to_sec()} segundos")

        if self.last_message_seq is not None:
            lost_msgs =  data.header.seq - self.last_message_seq -1
            if lost_msgs >0:
                rospy.loginfo(f"Se han perdido {lost_msgs} paquetes")

        self.last_message_seq = data.header.seq

        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.frame = image

    
    def __run_ia(self, frame: np.ndarray) -> list:
        """Function to segment a frame with own IA

        Args:
            frame (np.ndarray): Frame to predict

        Returns:
            list: List of result (masks, boxes, image) that return prediction in model YOLOv8
        """
        return self.model_ia.predict(source=frame, conf=self.predict_ia_conf, device='cpu')


def signal_handler(signal, frame) -> None:
    rospy.logwarn("Ctrl+C detected, stopping goal point thread...")
    image_process.running = False

        
if __name__ == '__main__':
    try:
        ## Absolute path
        file_path = os.path.abspath(__file__)
        ## Absolute path of folder models
        models_dir = os.path.join(os.path.dirname(file_path), 'models')
        ## Model IA path
        model_path = os.path.join(models_dir, 'model_16.pt')
        ## Call Process Image class
        image_process = ProcessImage(model_ia_path=model_path)
        ## Manage SIGINT signal
        signal.signal(signal.SIGINT, signal_handler)
        ## Upload thread
        list_threads = [image_process]
        ## Start and Join threads
        [wire.start() for wire in list_threads]
        [wire.join() for wire in list_threads]

    except rospy.ROSInterruptException:
        rospy.loginfo("Close this program")
        pass