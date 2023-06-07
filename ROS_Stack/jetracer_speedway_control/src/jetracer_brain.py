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
from jetracer_speedway_msgs.msg import Dictionary, KeyValue, Points, Velocities
from jetracer_speedway_srvs.srv import DepthToPoint, DepthToPointRequest
import threading as th
import numpy as np
import signal
import yaml


class Brain(th.Thread):


    def __init__(self) -> "Brain":
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

        ########################### Point ###########################
        self.points = None

        ################### Objects Dictionary ######################
        self.obj_dict = None
        self.class_list = ["stop sign",
                           "car"]
        
        self.is_going_slow = False ## TODO
        self.last_car_distance = None
        self.current_car_distance = None

        ########################### ROS ###########################
         ## Constants
        self.name_sub = config["navigation"]["pub_name_img"]
        self.name_sub_obj = config["navigation"]["pub_name_obj"]        
        self.name_pub = config["control"]["pub_name"]
        self.name_ros_node = config["control"]["node_name"]
        self.name_srv = config["sensors"]["srv_name"]
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)
        ## Publisher and Subscribers
        self.pub_vels = rospy.Publisher(self.name_pub, Velocities, queue_size=1)
        
        ## Wait for subscribers
        rospy.wait_for_message(self.name_sub, Points)
        self.sub_point = rospy.Subscriber(self.name_sub, Points, self.__callback_point)

        rospy.wait_for_message(self.name_sub_obj, Dictionary)
        self.sub_dictionary = rospy.Subscriber(self.name_sub_obj, Dictionary, self.__callback_dictionary)
        
        # Wait for service
        rospy.wait_for_service(self.name_srv)
        self.distance_srv = rospy.ServiceProxy(self.name_srv, DepthToPoint)


    def run(self) -> None:
        while self.running:
            ## Wait for a frame
            if self.points is None:
                self.rospyRate.sleep()
                continue
            
            ## Check if there is any car to follow
            goal_point = self.__check_car2follow(self.points)
            ## Pass desired point to angle
            angle = self.__convert_point_2_vel(goal_point)
            ## Publish desired point
            self.publish_vels(angle)

            rospy.loginfo(f"Angle: {angle}")
            self.rospyRate.sleep()


    def publish_vels(self, angle_deg: float) -> None:
            """Void to publish angular vel and linear vel in Twist format. The topic to publish is "jetracer_vels"

            Args:
                angle_deg (float): Angle
            """
            CONSTANT_VEL = 0.6
            vel = 0.05
            
            ## if not same:
            if angle_deg > 0:
                vel = 4.4*(1/abs(angle_deg))

            else:
                vel = CONSTANT_VEL
            
            ## Set value of vel if there is a car to follow
            if self.current_car_distance is not None:
                vel = self.current_car_distance
                self.last_car_distance = self.current_car_distance

            ## Check if stop_signal
            vel = self.__reduce_vel(vel)

            ## Break velocity
            vel = vel if vel <= CONSTANT_VEL else CONSTANT_VEL

            ## Create Twist message
            vel_msg = Velocities()
            vel_msg.angular = angle_deg
            vel_msg.linear = vel
            
            ## Publish message in topic vels_jetracer
            self.pub_vels.publish(vel_msg)


    def __callback_dictionary(self, msg: Dictionary) -> None:
        self.obj_dict = msg 


    def __callback_point(self, msg: Points) -> None:
        ## rospy.loginfo(msg)
        self.points = msg


    def __reduce_vel(self, vel: float) -> float:
        """Function to know if there is a stop. 
        If there is, the speed will be set based on the proximity to the stop 

        Args:
            vel (float): Current value of vel

        Returns:
            float: new vel value. If there isn't object return vel param
        """
        
        try:
            all_detections = self.obj_dict.dict
            for item in all_detections:
                if item.key == self.class_list[0]:
                    ## Call service to take distance to the point
                    list_dists = []
                    for point in item.value:
                        request = DepthToPointRequest()
                        request.x = point.x
                        request.y = point.y
                        distance = self.distance_srv(request)
                        list_dists.append[distance]

                    return min(list_dists)
                    
        except: pass

        return vel


    def __check_car2follow(self, current_pt: Points) -> Points:
        """Function to know if there is a car to follow. 
        If there is, the speed will be set based on the proximity to the car,
        set the goal_point with the car position. 

        Args:
            current_pt (Points): Current goal point

        Returns:
            Points: Return the new desired point to go. If there isn't cars return the same value
                    of current_pt param
        """
        try:
            all_detections = self.obj_dict.dict
            for item in all_detections:
                if item.key == self.class_list[1]:
                    ## Call service to take distance to the car
                    list_dists = []
                    for point in item.value:
                        request = DepthToPointRequest()
                        request.x = point.x
                        request.y = point.y
                        distance = self.distance_srv(request)
                        list_dists.append[distance]

                    index_min = list_dists.index(min(list_dists))
                    point = item.value[index_min]
                    current_pt.x = point.x
                    current_pt.y = point.y
                    self.current_car_distance = min(list_dists)
                    return current_pt

        except: pass
        self.current_car_distance = None
        return current_pt


    def __convert_point_2_vel(self, points: Points) -> float:
            """Function to extract angle from a goal point respect real point.
                To calculate angle use trigonometry.
                    - Calculate horizontal side
                    - Calculate vertical side
                    - Calculate angle with arctang
                    - To select the turn sign i do the next:
                        If go to right, that is, the desired point is to the right of the real point the sign is negative.
                        In the other case is positive.

            Args:
                goal_point (Point): Desired Point calculate below

            Returns:
                float: Return angle in degrees
            """

            ## Calculate sides
            horizontal_side = points.xr - points.x
            vertical_side = points.yr - points.y
            ## Take sign of turn
            sign = 1 if horizontal_side >= 0 else -1
            ## Calculate theta
            theta_rad = np.arctan2(abs(horizontal_side), vertical_side)
            theta_degree = theta_rad * 180.0 / np.pi
            ## Bounded theta for max value to jetracer
            theta_deg_bounded = theta_degree * 32.2 / 80

            return theta_deg_bounded*sign


def signal_handler(signal, frame):
    rospy.logwarn("Ctrl+C detected, stopping threads...")
    driver_control.running = False


if __name__ == '__main__':
    try:
        ## Call JetRacer and Brain classes
        driver_control = Brain()
        ## Manage SIGINT signal
        signal.signal(signal.SIGINT, signal_handler)
        ## Upload threads
        list_threads = [driver_control]
        ## Start and Join threads
        [wire.start() for wire in list_threads]
        [wire.join() for wire in list_threads]

    except rospy.ROSInterruptException:
        rospy.loginfo("Close this program")
        pass