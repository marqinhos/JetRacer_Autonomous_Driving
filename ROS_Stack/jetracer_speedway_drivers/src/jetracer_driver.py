#!/usr/bin/env python3

from .utils.jetracer_cls import JetRacer

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import threading as th
import signal
import yaml


class Driver(th.Thread):

    def __init__(self, jetracer: JetRacer) -> "Driver":
        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)
        ########################### YAML ###########################
        # Load config.yaml
        with open(rospy.get_param("/jetracer_speedway_bringup/config_file"), 'r') as f:
            config = yaml.safe_load(f)

        ########################### ROS ###########################
        ## Constants
        self.name_sub_vels = config["navigation"]["pub_name_img"]
        self.name_sub_emergency = config["navigation"]["pub_name_obs"]
        self.name_ros_node = config["driver"]["node_name"]
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        self.vels_sub = rospy.Subscriber(self.name_sub_vels, Twist, self.__callback_vels)
        self.emer_sub = rospy.Subscriber(self.name_sub_emergency, Bool, self.__callback_emergency)
        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)

        ########################### JetRacer ###########################
        self.jetracer = jetracer
        self.vel = 0
        self.angle = 0

        ########################### Running ###########################
        self.running = True
        self.emergency_stop = False


    def run(self) -> None:
        """Void to set velocities into JetRacer
        """
        rospy.loginfo("Setting velocities to JetRacer...")
        while self.running:
            if not self.emergency_stop:
                self.jetracer.set_vel(self.vel)
                self.jetracer.set_angle(self.angle)
                self.rospyRate.sleep()


    def __callback_vels(self, msg: Twist) -> None:
        """Callback to get the linear and angular vels, to set in JetRacer

        Args:
            msg (Twist): Message to get to subscribe to the jetracer_vels topic
        """
        self.vel = msg.linear.x
        self.angle = msg.angular.z


    def __callback_emergency(self, msg: Bool) -> None:
        """Callback to set emergency stop to JetRacer

        Args:
            msg (Bool): Message to get to subscribe to the jetracer_obstacle_detected
        """
        if msg.data is True:
            rospy.loginfo("EMERGENCY STOP")
            self.emergency_stop = True
            self.jetracer.stop_emergency()
        
        elif msg.data is False:
            if self.jetracer.run_stop_emergency is True:
                self.emergency_stop = False
                self.jetracer.run_stop_emergency = False 
        else: 
            pass


def signal_handler(signal, frame):
    rospy.logwarn("Ctrl+C detected, stopping threads...")
    driver_control.running = False
    driver_control.jetracer.stop()


if __name__ == '__main__':
    try:
        ## Call JetRacer and Brain classes
        jetracer = JetRacer()
        driver_control = Driver(jetracer)
        ## Manage SIGINT signal
        signal.signal(signal.SIGINT, signal_handler)
        ## Set rate to JetRacer
        jetracer.set_rate(driver_control.rospyRate)
        ## Upload threads
        list_threads = [jetracer, driver_control]
        ## Start and Join threads
        [wire.start() for wire in list_threads]
        [wire.join() for wire in list_threads]

    except rospy.ROSInterruptException:
        rospy.loginfo("Close this program")
        pass

