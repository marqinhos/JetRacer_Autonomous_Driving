#!/usr/bin/env python3


import pyrealsense2.pyrealsense2 as rs
import numpy as np
import threading as th
import cv2
import rospy
import sys
import yaml
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from jetracer_speedway_srvs.srv import DepthToPoint, DepthToPointResponse


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
        self.depth_pyrealsense = None

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
            rospy.logerr("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        ## Start streaming
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
                self.depth_pyrealsense = depth_frame
                self.color_frame = color_image
                self.depth_frame = depth_image

        except Exception as e:
            rospy.loginfo("Close Camera")    

        finally:
            rospy.loginfo("Close Camera")    
            pipeline.stop()


    def get_color_frame(self) -> np.ndarray:
        """Function to get the color image save in the buffer

        Returns:
            np.ndarray: Return the last color image save in the buffer
        """
        return self.color_frame


    def get_depth_frame(self) -> np.ndarray:
        """Function to get depth frame save in the buffer

        Returns:
            np.ndarray: Return the last depth frame save in the buffer
        """
        return self.depth_frame
    
    def get_pyrealsense_depth_frame(self) -> rs.depth_frame:
        """Function to get depth frame save in the buffer in original format

        Returns:
            pr.depth_frame: Return the last depth frame save in the buffer
        """
        return self.depth_pyrealsense


class ImageDepthPublisher(th.Thread):
    """Class to publish in a topic cam color image and in another topic depth frame"""

    def __init__(self, camera: IntelRealSense) -> "ImageDepthPublisher":
        ########################### Threads ###########################
        # Initialize thread
        th.Thread.__init__(self)

        ########################### YAML ###########################
        # Load config.yaml
        yaml_path = rospy.get_param('config_file')
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        ########################### IMAGE ###########################
        self.bridge = CvBridge()
        self.camera = camera

        ########################### OS ###########################
        self.running = True

        ########################### ROS ###########################
        ## Constants
        self.name_pub_img = config["sensors"]["pub_name_img"]
        self.name_pub_depth = config["sensors"]["pub_name_depth"]
        self.name_srv = config["navigation"]["srv_name"]

        self.name_ros_node = config["sensors"]["node_name_img_depth_intel"]
        ## Initialize node of ros
        rospy.init_node(self.name_ros_node)
        self.pub_image = rospy.Publisher(self.name_pub_img, Image, queue_size=1)
        self.pub_depth = rospy.Publisher(self.name_pub_depth, Image, queue_size=1)
        self.srv = rospy.Service(self.name_srv, DepthToPoint, self.__handle_request)

        ## Rate
        self.rate = config["rate"]
        self.rospyRate = rospy.Rate(self.rate)



    def run(self) -> None:
        rospy.loginfo("Publishing Image...")

        while not rospy.is_shutdown() and self.running:
            try:
                # print(self.camera.get_color_frame())
                ## Publish Color Image
                try:
                    self.__publish_frame(self.pub_image)
                except: assert ValueError(1)
                ## Publish Depth Image
                try:
                    pass
                    ## self.__publish_frame(self.pub_depth, False)
                except: assert ValueError(2)

            except Exception as err:
                if err.args[0] == 1: rospy.logwarn("Image is not Published")
                else: rospy.logwarn("Depth is not Published")
                
            self.rospyRate.sleep()
            
        ## Close camera
        self.camera.running = False
        rospy.loginfo("Shutdown")


    def __handle_request(self, request: DepthToPoint) -> DepthToPointResponse:
        frame = self.camera.get_pyrealsense_depth_frame()
        color_frame = self.camera.get_color_frame()
        if frame is None:
            rospy.logwarn("No image")
            return DepthToPointResponse(distance=0.0)

        x = request.x
        y = request.y

        if x < 0 or x >= color_frame.shape[1] or y < 0 or y >= color_frame.shape[0]:
            rospy.logwarn("Point out image")
            return DepthToPointResponse(distance=0.0)

        distance = frame.get_distance(x, y)

        return DepthToPointResponse(distance=float(distance))

    
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
