#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

def scan_callback(msg):
    
    MIN_CENTER = .16
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
    rospy.init_node('obstacle_detector')
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    obstacle_pub = rospy.Publisher('/jetracer_obstacle_detected', Bool, queue_size=1)
    rospy.spin()