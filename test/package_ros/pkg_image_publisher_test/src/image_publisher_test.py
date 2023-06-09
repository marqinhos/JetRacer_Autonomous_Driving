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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import depthai as dai
import time
import os 
import threading as th
import sys


class Camera_OAK(th.Thread):

    def __init__(self) -> "Camera_OAK":
        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)

        ########################### IMAGE ###########################
        self.width = 640
        self.height = 480
        self.frame = np.zeros((self.width, self.height))

        ########################### OS ###########################
        self.running = True

        ########################### CAMERA ###########################
        self.device = None


    @staticmethod
    def __show_specs(device) -> None:
        print('Device name:', device.getDeviceName())
        # Bootloader version
        if device.getBootloaderVersion() is not None:
            print('Bootloader version:', device.getBootloaderVersion())
        # Print out usb speed
        print('Usb speed:', device.getUsbSpeed().name)
        # Connected cameras
        print('Connected cameras:', device.getConnectedCameraFeatures())


    def run(self) -> None:
        try:
            with dai.Device() as device:
                self.device = device
                self.__show_specs(device)
                # Create pipeline
                pipeline = dai.Pipeline()
                cams = device.getConnectedCameraFeatures()
                streams = []
                
                for cam in cams:
                    """Initialize camera
                    """
                    print(str(cam), str(cam.socket), cam.socket)
                    c = pipeline.create(dai.node.Camera)
                    x = pipeline.create(dai.node.XLinkOut)
                    c.isp.link(x.input)
                    c.setBoardSocket(cam.socket)
                    stream = str(cam.socket)
                    if cam.name:
                        stream = f'{cam.name} ({stream})'
                    x.setStreamName(stream)
                    streams.append(stream)
                
                # Start pipeline
                device.startPipeline(pipeline)
                fpsCounter = {}

                while not device.isClosed() and self.running:
                    if self.running is False:
                        raise Exception
                    queueNames = device.getQueueEvents(streams)
                    for stream in queueNames:
                        messages = device.getOutputQueue(stream).tryGetAll()
                        fpsCounter[stream] = fpsCounter.get(stream, 0.0) + len(messages)
                        for message in messages:
                            # Display arrived frames
                            if type(message) == dai.ImgFrame:                            
                                frame = message.getCvFrame()
                                resize_points = (self.width, self.height)
                                frame = cv2.resize(frame, resize_points, interpolation= cv2.INTER_LINEAR)
                                self.frame = frame
        except Exception as e:
            rospy.loginfo("Close Camera")
            device.close()

        finally:
            rospy.loginfo("Close Camera")
            device.close()


    def get_frame(self) -> np.ndarray:
        """Function to get the image save in the buffer

        Returns:
            np.ndarray: Return the last image save in the buffer
        """
        return self.frame


class ImagePublisher(th.Thread):
    """Class to publish in a topic cam image"""

    def __init__(self, camera: Camera_OAK, name_ros_node: str="image_publisher", name_pub: str="jetracer_image") -> "ImagePublisher":
        
        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)

        ########################### ROS ###########################
        ## Constants
        self.name_pub = name_pub
        self.name_ros_node = name_ros_node
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        self.pub_image = rospy.Publisher(self.name_pub, Image, queue_size=1)
        ## Rate
        self.rate = 10
        self.rospyRate = rospy.Rate(self.rate)

        ########################### IMAGE ###########################
        self.bridge = CvBridge()
        self.camera = camera

        ########################### OS ###########################
        self.running = True


    def run(self) -> None:
        rospy.loginfo("Publishing Image...")

        while not rospy.is_shutdown() and self.running:
            try:
                self.__publish_image()

            except: 
                rospy.logwarn("Image is not Published")
            
            self.rospyRate.sleep()
            
        ## Close camera
        self.camera.running = False
        rospy.loginfo("Shutdown")


    def __publish_image(self) -> None:
        """Void to publish image
        """
        image = self.camera.get_frame()
        ## Check image
        if image is not None: 
            ## Convert opencv image to image msg to publish
            ros_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.pub_image.publish(ros_image)


def signal_handler(signal, frame) -> None:
    print("Ctrl+C detected, stopping threads...")
    image_publisher.running = False
    sys.exit(0)


if __name__ == '__main__':
    try:
        camera = Camera_OAK()
        image_publisher = ImagePublisher(camera)
        list_run = [camera, image_publisher]
        [item.start() for item in list_run]
        [item.join() for item in list_run]

    except rospy.ROSInterruptException:
        pass

