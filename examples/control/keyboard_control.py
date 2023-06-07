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
#import keyboard 
import rospy
from pynput import keyboard

class Connect:

    def __init__(self):
        self.i2c_address = 0x40	
        self.steering_channel = 0
        self.throttle_channel = 1
        self.kit = ServoKit(channels=16, address=self.i2c_address)
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        self.throttle_motor = self.kit.continuous_servo[self.throttle_channel]

        self.running = True
        self.rate = rospy.Rate(10)
        
        self.vel = 0
        self.turn = 0

    def __call__(self):
        """Function to control JetRacer. For use:
            -> w: to go forward, or increased velocity
            -> s: to go back, or decreased velocity
            -> a: to turn left, or turnning left
            -> d: to turn right, or turnning right
        """
        while self.running:
            print("SCAN")
            with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
                listener.join()
            # keyboard.press('s') is_pressed
            
                
            self.steering_motor.throttle = self.turn
            self.throttle_motor.throttle = self.vel
            # print(self.vel)
            
            self.rate.sleep()

    def on_press(self, key):
        try:
            #print('alphanumeric key {0} pressed'.format(key.char))
            self.aux_key(key.char)
            return False
        except AttributeError:
            print('special key {0} pressed'.format(
            key))


    def on_release(self,key):
        #print('{0} released'.format(key))
        if key == keyboard.Key.esc:
            # Stop listener
            self.running = False
            return False
        
    def aux_key(self, letter):
        #if keyboard.press('q'): 
        if letter == 'q':
            print('You Pressed Q Key!')
            self.running = False  # finishing the loop
        #if keyboard.press('w'):
        if letter == 'w':
            self.vel = self.vel + 0.1 if self.vel+0.1 <= 1.0 else 1
            
        #if keyboard.press('s'):
        if letter == 's':
            self.vel = self.vel - 0.1 if abs(self.vel-0.1) <= 1.0 else -1
            
        #if keyboard.press('d'):
        if letter == 'a':
            self.turn = self.turn + 0.1 if self.turn+0.1 <= 1.0 else  1
            
        #if keyboard.press('a'):
        if letter == 'd':
            self.turn = self.turn - 0.1 if abs(self.turn-0.1) <= 1.0 else  -1

class Stop(Connect):
    def __init__(self):
        super().__init__()

    def __call__(self):
        """Function to stop all motors
        """
        self.steering_motor.throttle = 0.0
        self.throttle_motor.throttle = 0.0




if __name__ == '__main__':
	
	rospy.init_node('key_control')
	
	Stop()()
	
	Connect()()
	
	Stop()()