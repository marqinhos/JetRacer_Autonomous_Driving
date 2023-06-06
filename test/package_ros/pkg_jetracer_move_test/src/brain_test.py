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


from jetracer_test import JetRacer

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import threading as th
import signal


class Brain(th.Thread):

    def __init__(self, jetracer: JetRacer, name_sub_vels: str="jetracer_vels", name_sub_emergency: str="jetracer_obstacle_detector", name_ros_node: str="vels_subscriber") -> "Brain":
        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)

        ########################### ROS ###########################
        ## Constants
        self.name_sub_vels = name_sub_vels
        self.name_sub_emergency = name_sub_emergency
        self.name_ros_node = name_ros_node
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        self.vels_sub = rospy.Subscriber(self.name_sub_vels, Twist, self.__callback_vels)
        self.emer_sub = rospy.Subscriber(self.name_sub_emergency, Bool, self.__callback_emergency)
        ## Rate
        self.rate = 10
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
    brain_control.running = False
    brain_control.jetracer.stop()


if __name__ == '__main__':
    try:
        ## Call JetRacer and Brain classes
        jetracer = JetRacer()
        brain_control = Brain(jetracer)
        ## Manage SIGINT signal
        signal.signal(signal.SIGINT, signal_handler)
        ## Set rate to JetRacer
        jetracer.set_rate(brain_control.rospyRate)
        ## Upload threads
        list_threads = [jetracer, brain_control]
        ## Start and Join threads
        [wire.start() for wire in list_threads]
        [wire.join() for wire in list_threads]

    except rospy.ROSInterruptException:
        rospy.loginfo("Close this program")
        pass

