#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def scan_callback(msg):
    # Angle to start
    start_angle = msg.angle_min
    end_angle = -0.5
    # Index to start
    start_index = int((start_angle - msg.angle_min) / msg.angle_increment)
    end_index = int((end_angle - msg.angle_min) / msg.angle_increment)
    
    # Calcular el promedio de los valores en ese rango de ángulos
    avg_value = sum(msg.ranges[start_index:end_index]) / (end_index - start_index)
    # Detectar obstáculos en el lado derecho
    if avg_value < 1.0:
        rospy.loginfo("Obstacle detected on the right side")
        obstacle_pub.publish(True)
    else:
        obstacle_pub.publish(False)

if __name__ == '__main__':
    rospy.init_node('obstacle_detector')
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    obstacle_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=1)
    rospy.spin()