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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import yaml
import threading as th
import signal


class SecurityStop(th.Thread):
     
    def __init__(self) -> "SecurityStop":
        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)

        ########################### YAML ###########################
        # Load config.yaml
        yaml_path = rospy.get_param('config_file')
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        ########################### DISTANCES ###########################
        ## Set distance constants
        self.MIN_CENTER = config["navigation"]["min_center"]
        self.MIN_BACK = config["navigation"]["min_back"]
        self.MIN_SIDES = config["navigation"]["min_sides"]

        ########################### SCAN ###########################
        self.ranges = None

        ########################### Running ###########################
        self.running = True

        ########################### ROS ###########################
        ## Constants
        self.name_ros_node = config["navigation"]["node_name_obs"]
        self.name_sub = config["sensors"]["pub_name_rplidar"]
        self.name_pub = config["navigation"]["pub_name_obs"]
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)
        self.obstacle_pub = rospy.Publisher(self.name_pub, Bool, queue_size=1)
        rospy.wait_for_message(self.name_sub, LaserScan)
        self.scan_sub = rospy.Subscriber(self.name_sub, LaserScan, self.__callback_scan)


    def run(self) -> None:
        while self.running:
            if self.ranges is None:
                self.rospyRate.sleep()
                continue
            
            self.publish_warning(self.ranges)
            self.rospyRate.sleep()


    def publish_warning(self, ranges:list) -> None:
        laser_beam_c = ranges[0:80] + ranges[len(ranges)-80:len(ranges)]
        laser_beam_l = ranges[80:280]
        laser_beam_b = ranges[280:440]
        laser_beam_r = ranges[440:640]
        # laser_beam_c = [0.1 if laser_beam_c[i] > 99 else laser_beam_c[i] for i in range(len(laser_beam_c))]
        # mean_center = sum(laser_beam_c)/len(laser_beam_c)
        smallest_10 = sorted(laser_beam_c)[:10]
        mean = sum(smallest_10) / len(smallest_10)
        if mean <= self.MIN_CENTER:
            rospy.loginfo("OBSTACLE IN FRONT")
            self.obstacle_pub.publish(True)
        else:
            self.obstacle_pub.publish(False)

    def __callback_scan(self, msg: LaserScan) -> None:
        self.ranges = msg.ranges


def signal_handler(signal, frame):
    rospy.logwarn("Ctrl+C detected, stopping emergency stop thread...")
    security.running = False



if __name__ == '__main__':
    try:
        ## Call JetRacer and Brain classes
        security = SecurityStop()
        ## Manage SIGINT signal
        signal.signal(signal.SIGINT, signal_handler)
        ## Upload threads
        list_threads = [security]
        ## Start and Join threads
        [wire.start() for wire in list_threads]
        [wire.join() for wire in list_threads]

    except rospy.ROSInterruptException:
        rospy.loginfo("Close this program")
        pass
 

