#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

def scan_callback(msg: LaserScan):
    """Callback to get all ranges that rplidar return

    Args:
        msg (LaserScan): Message that subscriber to rplidar return
    """
    
    MIN_CENTER = .24
    MIN_BACK = .18
    MIN_SIDES = .17


    laser_beam_c = msg.ranges[0:80] + msg.ranges[len(msg.ranges)-80:len(msg.ranges)]
    laser_beam_l = msg.ranges[80:280]
    laser_beam_b = msg.ranges[280:440]
    laser_beam_r = msg.ranges[440:640]

    if min(laser_beam_c) <= MIN_CENTER or min(laser_beam_b) <= MIN_BACK or min(laser_beam_r+laser_beam_l) <= MIN_SIDES:
        rospy.loginfo("Obstacle detected AUTOMATIC STOP")
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
 

