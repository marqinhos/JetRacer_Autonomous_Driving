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

def scan_callback(msg: LaserScan):
    """Callback to get all ranges that rplidar return

    Args:
        msg (LaserScan): Message that subscriber to rplidar return
    """
    
    MIN_CENTER = .31
    MIN_BACK = .18
    MIN_SIDES = .17


    laser_beam_c = msg.ranges[0:80] + msg.ranges[len(msg.ranges)-80:len(msg.ranges)]
    laser_beam_l = msg.ranges[80:280]
    laser_beam_b = msg.ranges[280:440]
    laser_beam_r = msg.ranges[440:640]
    # laser_beam_c = [0.1 if laser_beam_c[i] > 99 else laser_beam_c[i] for i in range(len(laser_beam_c))]
    # mean_center = sum(laser_beam_c)/len(laser_beam_c)
    smallest_10 = sorted(laser_beam_c)[:10]
    mean = sum(smallest_10) / len(smallest_10)
    if mean <= MIN_CENTER:
        rospy.loginfo("OBSTACLE IN FRONT")
        obstacle_pub.publish(True)
    else:
        obstacle_pub.publish(False)

if __name__ == '__main__':

    ########################### ROS ###########################
    ## Constants
    name_ros_node = "obstacle_detector"
    name_sub = "/scan"
    name_pub = "jetracer_obstacle_detector"
    ## Initialize node of ros
    rospy.init_node(name_ros_node)
    scan_sub = rospy.Subscriber(name_sub, LaserScan, scan_callback)
    obstacle_pub = rospy.Publisher(name_pub, Bool, queue_size=1)
    ## Running
    rospy.spin()
 

