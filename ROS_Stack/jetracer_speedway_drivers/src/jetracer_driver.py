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


from utils import JetRacer

import rospy
from jetracer_speedway_msgs.msg import Velocities
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
        yaml_path = rospy.get_param('config_file')
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        ########################### JetRacer ###########################
        self.jetracer = jetracer
        self.vel = 0
        self.angle = 0

        ########################### Running ###########################
        self.running = True
        self.emergency_stop = False

        ########################### ROS ###########################
        ## Constants
        self.name_sub_vels = config["control"]["pub_name"]
        self.name_sub_emergency = config["navigation"]["pub_name_obs"]
        self.name_ros_node = config["driver"]["node_name"]
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)
        ## Publisher and Subscribers
        rospy.wait_for_message(self.name_sub_vels, Velocities)
        self.vels_sub = rospy.Subscriber(self.name_sub_vels, Velocities, self.__callback_vels)
        rospy.wait_for_message(self.name_sub_emergency, Bool)
        self.emer_sub = rospy.Subscriber(self.name_sub_emergency, Bool, self.__callback_emergency)


    def run(self) -> None:
        """Void to set velocities into JetRacer
        """
        rospy.loginfo("Setting velocities to JetRacer...")
        while self.running:
            if not self.emergency_stop:
                self.jetracer.set_vel(self.vel)
                self.jetracer.set_angle(self.angle)
                self.rospyRate.sleep()
            
            else:
                self.jetracer.set_vel(0)
                self.jetracer.set_angle(0)
                self.rospyRate.sleep()
                
        self.jetracer.set_vel(0)
        self.jetracer.set_angle(0)


    def __callback_vels(self, msg: Velocities) -> None:
        """Callback to get the linear and angular vels, to set in JetRacer

        Args:
            msg (Twist): Message to get to subscribe to the jetracer_vels topic
        """
        self.vel = msg.linear
        self.angle = msg.angular


    def __callback_emergency(self, msg: Bool) -> None:
        """Callback to set emergency stop to JetRacer

        Args:
            msg (Bool): Message to get to subscribe to the jetracer_obstacle_detected
        """
        if msg.data is True:
            rospy.loginfo("EMERGENCY STOP")
            self.emergency_stop = True
            ## self.jetracer.stop_emergency()
        
        elif msg.data is False:
            if self.emergency_stop is True:
                self.emergency_stop = False
                ## self.jetracer.run_stop_emergency = False 
        else: 
            pass


def signal_handler(signal, frame):
    rospy.logwarn("Ctrl+C detected, stopping threads...")
    driver_control.jetracer.set_vel(0)
    driver_control.jetracer.set_angle(0)
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

