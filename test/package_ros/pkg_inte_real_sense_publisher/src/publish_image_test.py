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


import pyrealsense2 as rs
import numpy as np
import threading as th
import cv2
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class IntelRealSense(th.Thread):

    def __init__(self) -> "IntelRealSense":
        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)

        ########################### IMAGE ###########################
        self.width = 640
        self.height = 480
        self.color_frame = np.zeros((self.width, self.height))
        self.depth_frame = np.zeros((self.width, self.height))

        ########################### OS ###########################
        self.running = True


    def run(self):
        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)

        try:
            while self.running:

                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                self.color_frame = color_image
                self.depth_frame = depth_image

        finally:

            # Stop streaming
            print("Closed camera")
            pipeline.stop()


    def get_color_frame(self):
        return self.color_frame


    def get_depth_frame(self):
        return self.depth_frame


class ImageDepthPublisher(th.Thread):

    def __init__(self, camera: IntelRealSense) -> "ImageDepthPublisher":
        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)
        
        ########################### IMAGE ###########################
        self.bridge = CvBridge()
        self.camera = camera

        ########################### OS ###########################
        self.running = True

        ########################### ROS ###########################
        ## Constants
        self.name_pub_img = "jetracer_img_pub"
        self.name_pub_depth = "jetracer_depth_pub"

        self.name_ros_node = "jetracer_intel_node"
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        self.pub_image = rospy.Publisher(self.name_pub_img, Image, queue_size=1)
        self.pub_depth = rospy.Publisher(self.name_pub_depth, Image, queue_size=1)

        ## Rate
        self.rate = 10
        self.rospyRate = rospy.Rate(self.rate)



    def run(self) -> None:
        rospy.loginfo("Publishing Image...")

        while not rospy.is_shutdown() and self.running:
            try:
                # print(self.camera.get_color_frame())
                ## Publish Color Image
                self.__publish_frame(self.pub_image)
                ## Publish Depth Image
                self.__publish_frame(self.pub_depth, False)

            except: 
                rospy.logwarn("Image is not Published")
            
            self.rospyRate.sleep()
            
        ## Close camera
        self.camera.running = False
        rospy.loginfo("Shutdown")

    
    def __publish_frame(self, pub: rospy.Publisher, img: bool=True) -> None:
        if img:
            image = self.camera.get_color_frame()
            if image is not None:
                ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
                pub.publish(ros_image)
        else:
            depth = self.camera.get_depth_frame()
            if depth is not None:
                ros_depth = self.bridge.cv2_to_imgmsg(depth)
                pub.publish(ros_depth)


def signal_handler(signal, frame) -> None:
    print("Ctrl+C detected, stopping threads...")
    image_publisher.running = False
    sys.exit(0)


if __name__ == '__main__':
    try:
        camera = IntelRealSense()
        image_publisher = ImageDepthPublisher(camera)
        list_run = [camera, image_publisher]
        [item.start() for item in list_run]
        [item.join() for item in list_run]

    except rospy.ROSInterruptException:
        pass
