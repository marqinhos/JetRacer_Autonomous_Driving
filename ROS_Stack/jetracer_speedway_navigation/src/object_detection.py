#!/usr/bin/env python3

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
        self.predict_ia_conf = config["confidence"]

        ########################### IMAGE ###########################
        self.frame = None
        self.bridge = CvBridge()

        ########################### ROS ###########################
        ## Constants
        self.name_sub = config["sensors"]["pub_name_img"]
        self.name_pub = config["navigation"]["pub_name_img"]        ## TODO
        self.name_ros_node = config["navigation"]["node_name_img"]  ## TODO
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)
        ## Publisher and Subscribers
        self.pub_vels = rospy.Publisher(self.name_pub, Points, queue_size=1) ## TODO
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

            ## Get desired Point
            try:
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

            ## Publish desired point
            self.publish_point(desired_pt)
            # rospy.loginfo(f"Point: {desired_pt}")
            self.rospyRate.sleep()


        rospy.loginfo(f"Shutdown {self.name_ros_node}")


    def __run_ia(self, frame: np.ndarray) -> list:
        """Function to segment a frame with own IA

        Args:
            frame (np.ndarray): Frame to predict

        Returns:
            list: List of result (masks, boxes, image) that return prediction in model YOLOv8
        """
        return self.model_ia.predict(source=frame, conf=self.predict_ia_conf)
    

def signal_handler(signal, frame) -> None:
    rospy.logwarn("Ctrl+C detected, stopping threads...")
    image_process.running = False

        
if __name__ == '__main__':
    try:
        ## Absolute path
        file_path = os.path.abspath(__file__)
        ## Absolute path of folder models
        models_dir = os.path.join(os.path.dirname(file_path), 'models')
        ## Model IA path
        model_path = os.path.join(models_dir, 'yolov8n-seg.pt')
        ## Call Process Image class
        image_process = ObjectDetection(model_ia_path=model_path)
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
