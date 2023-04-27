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

# Import utils
from .utils.utils import *
# Import detections features
from .utils.image_process import Features_Detection


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
        with open(rospy.get_param("/jetracer_speedway_bringup/config_file"), 'r') as f:
            config = yaml.safe_load(f)

        ########################### ROS ###########################
        ## Constants
        self.name_sub = config["sensors"]["pub_name_img"]
        self.name_pub = config["navigation"]["offsetright"]
        self.name_ros_node = config["navigation"]["node_name_img"]
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        self.sub_img = rospy.Subscriber(self.name_sub, Image, self.__callback_image)
        self.pub_vels = rospy.Publisher(self.name_pub, Twist, queue_size=1)
        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)

        ########################### Running ###########################
        self.running = True

        ########################### IA ###########################
        self.model_ia_path = model_ia_path
        self.model_ia = YOLO(self.model_ia_path)
        self.predict_ia_conf = config["confidence"]

        ########################### IMAGE ###########################
        self.frame = None
        self.bridge = CvBridge()

        #################### FEATURES DETECTION ####################
        self.compute_detect = Features_Detection()
        self.last_desired_pt = Point(0, 0)


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

        rospy.loginfo(f"Shutdown {self.name_ros_node}")
        
    
    def publish_point(self, desired_point: Point) -> None:
        """Void to publish desired point in Points format. The topic to publish is "jetracer_vels"

        Args:
            angle_deg (float): Angle
        """
        # Create Points message
        point_msg = Points()
        point_msg.x = desired_point.x
        point_msg.y = desired_point.y
        
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

    
    def __callback_image(self, data: Image) -> None:
        """Callback to set in self variable the values of image. Convert the type of data to opencv image

        Args:
            data (Image): Image in format image msg 
        """
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.frame = image

    
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
        ## Model IA path
        model_path = os.path.join('.', 'models', 'best.pt')
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
