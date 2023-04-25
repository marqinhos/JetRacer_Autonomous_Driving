#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os 
import threading as th
import signal

# Import YOLO
from ultralytics import YOLO

# Import type of msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Import converter ros image to cv2 image
from cv_bridge import CvBridge

# Import utils
from utils import *


class ProccesImage(th.Thread):
    """Class to subscribe a topic to take image and by means of IA extract the lane and lines segmentation.
        Them process the segmentation, to extract the desired point to go the robot
    """

    def __init__(self, model_ia_path: str,name_sub: str="jetracer_image", name_pub: str="jetracer_vels", name_ros_node: str="procces_and_sub_image"):

        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)
        
        ########################### ROS ###########################
        ## Constants
        self.name_sub = name_sub
        self.name_pub = name_pub
        self.name_ros_node = name_ros_node
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        self.sub_img = rospy.Subscriber(self.name_sub, Image, self.__callback_image)
        self.pub_vels = rospy.Publisher(self.name_pub, Twist, queue_size=1)
        ## Rate
        self.rate = 10
        self.rospyRate = rospy.Rate(self.rate)

        ########################### Running ###########################
        self.running = True

        ########################### IA ###########################
        self.model_ia_path = model_ia_path
        self.model_ia = YOLO(self.model_ia_path)
        self.predict_ia_conf = .8

        ########################### IMAGE ###########################
        self.frame = None
        self.bridge = CvBridge()


    def run(self):
        rospy.loginfo("Processing image...")
        while self.running:
            ## Wait for a frame
            if self.frame is None:
                self.rospyRate.sleep()
                continue
            
            ## Execute ia to extrac features
            result = self.__run_ia(self.frame)

            ## Get desired Point

        rospy.loginfo(f"Shutdown {self.name_ros_node}")
        
    
    
    def __callback_image(self, data: Image) -> None:
        """Callback to set in self variable the values of image. Conver the type of data to opencv image

        Args:
            data (Image): Image in format image msg 
        """
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.frame = image

    
    def __run_ia(self, frame: np.ndarray) -> list:
        """Function to segment a frame with own IA

        Args:
            frame (np.darray): Frame to predict

        Returns:
            list: List of result (masks, boxes, image) that return prediction in model YOLOv8
        """
        return self.model_ia.predict(source=frame, conf=self.predict_ia_conf)


    def __convert_point_2_vel(self, goal_point: Point):
        pass


def signal_handler(signal, frame):
    print("Ctrl+C detected, stopping threads...")
    image_process.running = False

        
if __name__ == '__main__':
    try:
        
        model_path = os.path.join('.', 'models', 'best.pt')
        
        image_process = ProccesImage(model_ia_path=model_path)
        # Manejar señal SIGINT
        signal.signal(signal.SIGINT, signal_handler)

        list_threads = [image_process]
        [wire.start() for wire in list_threads]
        [wire.join() for wire in list_threads]

    except rospy.ROSInterruptException:
        pass
