#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import yaml


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

    ########################### YAML ###########################
    # Load config.yaml
    yaml_path = rospy.get_param('config_file')
    with open(yaml_path, 'r') as f:
           config = yaml.safe_load(f)

    ########################### ROS ###########################
    ## Constants
    name_ros_node = config["navigation"]["node_name_obs"]
    name_sub = config["sensors"]["pub_name_rplidar"]
    name_pub = config["navigation"]["pub_name_obs"]
    ## Initialize node of ros
    rospy.init_node(name_ros_node)
    obstacle_pub = rospy.Publisher(name_pub, Bool, queue_size=1)
    rospy.wait_for_message(name_sub, LaserScan)
    scan_sub = rospy.Subscriber(name_sub, LaserScan, scan_callback)
    ## Running
    rospy.spin()
 

