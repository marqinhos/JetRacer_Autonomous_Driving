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

# Import detections features
from extract_features import Features_Detection

class ProccesImage(th.Thread):
    """Class to subscribe a topic to take image and by means of IA extract the lane and lines segmentation.
        Them process the segmentation, to extract the desired point to go the robot
    """

    def __init__(self, model_ia_path: str,name_sub: str="jetracer_image", name_pub: str="jetracer_vels", name_ros_node: str="procces_and_sub_image") -> None:

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

        #################### FEATURES DETECTION ####################
        self.compute_detec = Features_Detection()
        self.last_desired_pt = Point(0, 0)


    def run(self) -> None:
        rospy.loginfo("Processing image...")
        while self.running:
            ## Wait for a frame
            if self.frame is None:
                self.rospyRate.sleep()
                continue
            
            ## Execute ia to extrac features
            result = self.__run_ia(self.frame)

            ## Get desired Point
            try:
                desired_pt = self.compute_detec.run(result)
                if not self.last_desired_pt.zero():
                    if abs(self.last_desired_pt.x - desired_pt.x) > 95 or abs(self.last_desired_pt.y - desired_pt.y) > 80:
                        rospy.logwarn("BAD DESIRED POINT")
                        desired_pt = self.last_desired_pt.middle_2_point(desired_pt)
                self.last_desired_pt = desired_pt
            except: 
                desired_pt = self.last_desired_pt

            ## Show Point in image
            self.__show(self.frame, desired_pt)

            ## Show angle to turn
            angle_degree = self.__convert_point_2_vel(desired_pt)
            self.publish_vels(angle_degree)
            rospy.loginfo(f"Angle: {angle_degree}")

        rospy.loginfo(f"Shutdown {self.name_ros_node}")
        
    
    def publish_vels(self, angle_deg: float) -> None:
            CONSTANT_VEL = 0.6
            vel = 0.05
            
            #if not same:
            if angle_deg > 0:
                vel = 4.4*(1/abs(angle_deg))  # Aquí se asigna un valor arbitrario para la velocidad

            else:
                vel = CONSTANT_VEL
            
            # Break velocity
            vel = vel if vel <= CONSTANT_VEL else CONSTANT_VEL
            # Crear un mensaje de tipo Twist y asignar los valores de angle y vel
            twist_msg = Twist()
            twist_msg.angular.z = angle_deg
            twist_msg.linear.x = vel
            
            # Publicar el mensaje en el topic vels_jetracer
            self.pub_vels.publish(twist_msg)

    def __show(self, image: np.ndarray, point: Point):
        cv2.circle(image, (point.x, point.y), 5, (0, 0, 255), -1)
        cv2.circle(image, (self.compute_detec.size[0]//2, self.compute_detec.size[1]-10), 5, (255, 0, 0), -1)
        cv2.imshow(self.name_ros_node , image)
        cv2.waitKey(3)

    
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


    def __convert_point_2_vel(self, goal_point: Point) -> float:

        real_pose = Point(self.compute_detec.size[0]//2, self.compute_detec.size[1]-10)
        ## Calculate sides
        horizontal_side = real_pose.x - goal_point.x
        vertical_side = real_pose.y - goal_point.y
        ## Take sign of turn
        sign = 1 if horizontal_side >= 0 else -1
        ## Calculate theta
        theta_rad = np.arctan2(abs(horizontal_side), vertical_side)
        theta_degree = theta_rad * 180.0 / np.pi
        ## Bounded theta for max value to jetracer
        theta_deg_bounded = theta_degree * 32.2 / 80

        return theta_deg_bounded*sign


def signal_handler(signal, frame) -> None:
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
