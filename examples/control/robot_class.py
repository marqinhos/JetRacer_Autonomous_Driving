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


from adafruit_servokit import ServoKit
import numpy as np
import rospy


class ConnectJetRacer():

    def __init__(self) -> None:
        """Initialition of variables
        """
        self.__i2c_address = 0x40
        self.__steering_channel = 0
        self.__throttle_channel = 1

        self.kit = ServoKit(channels=16, address=self.__i2c_address)

        self.steering_motor = self.kit.continuous_servo[self.__steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.__throttle_channel]

        self.rate = rospy.Rate(5)

class JetRacer(ConnectJetRacer):

    def __init__(self) -> None:
        super().__init__()

        self.__running = True

        self.__max_vel = 0.8
        self.__min_vel = 0.08
        self.__max_angle_turn = 32.2
        self.__min_angle_turn = 0

        self.current_value_vel = 0
        self.current_value_turn = 0

        self.circle_center = 0


    def run(self):
        """Process to run the program. Inside it only set the velocity and turn to adafruit motors.
            If there are some problems, they show into logerror ROS. 
        """
        while self.__running:
            ####################################################
            ## Maybe add pid controller to reduce delta error ##
            ####################################################
            try:
                self.steering_motor = self.current_value_turn
                self.throttle_motor = self.current_value_vel

                self.rate.sleep()
            except Exception as err:
                rospy.logerr(err.args)
        
        rospy.loginfo("Robot Node is closed")

    def stop(self):
        
        self.current_value_turn = 0
        self.current_value_vel = 0
        self.steering_motor = self.current_value_turn
        self.throttle_motor = self.current_value_vel
        self.__running = False


    def set_angle(self, angle: float) -> None:
        """Function to update the turn's value to new value 

        Args:
            new_turn (float): New value of turn. As default 0.0 to reset.

        """
        try:
            value_turn = angle / self.__max_angle_turn
            if abs(value_turn) > 1: 
                rospy.logwarn("Module of angle must be under 32.2º")
                value_turn = 1 if value_turn > 1 else -1

            self.current_value_turn = value_turn
        except Exception as err:
            rospy.logerr(err.args)
            raise err.args
        

    def set_vel(self, vel: float) -> None:
        """Function to update velocity value, to new value

        Args:
            new_vel (float): New value of velocity you want. As default 0.0 to reset.

        """
        try:
            value_vel = vel / self.__max_vel
            if abs(value_vel) > 1: 
                rospy.logwarn("Module of vel must be under 0.8 m/s")
                value_vel = 1 if value_vel > 1 else -1

            self.current_value_vel = value_vel
        except Exception as err:
            rospy.logerr(err.args)
            raise err.args
    

    def get_current_vel(self) -> float:
        """Function to get the current value of velocity

        Returns:
            float: Value of velocity
        """
        return self.current_value_vel * self.__max_vel
        

    def get_current_angle(self) -> float:
        """Function to get the current value of turn

        Returns:
            float: Value of turn
        """
        return self.current_value_turn * self.__max_angle_turn
    
