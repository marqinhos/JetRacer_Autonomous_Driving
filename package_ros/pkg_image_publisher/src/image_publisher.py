#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import depthai as dai
import time
import os 
import threading as th


class Camera_OAK(th.Thread):

    def __init__(self) -> None:
        super().__init__()
        self.width = 640
        self.height = 480
        self.time2save = 5
        self.__show = False
        self.root_folder = os.path.join(".", "data")
        self.frame = np.zeros((640, 480))


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
        with dai.Device() as device:
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
            lastFpsCount = {}
            tfps = time.time()
            start_time = tfps
            num_img = 0

            while not device.isClosed():
                queueNames = device.getQueueEvents(streams)
                for stream in queueNames:
                    messages = device.getOutputQueue(stream).tryGetAll()
                    fpsCounter[stream] = fpsCounter.get(stream, 0.0) + len(messages)
                    for message in messages:
                        # Display arrived frames
                        if type(message) == dai.ImgFrame:
                            # render fps
                            fps = lastFpsCount.get(stream, 0)
                            
                            frame = message.getCvFrame()
                            resize_points = (self.width, self.height)
                            frame = cv2.resize(frame, resize_points, interpolation= cv2.INTER_LINEAR)

                            self.frame = frame
                            

    def get_frame(self):
        return self.frame


class ImagePublisher(th.Thread):

    def __init__(self, camera):
        super().__init__()
        # Inicializar el nodo de ROS
        rospy.init_node('image_publisher')

        # Configurar el publicador de imágenes
        self.image_pub = rospy.Publisher('image', Image, queue_size=1)

        # Configurar el convertidor de OpenCV a ROS
        self.bridge = CvBridge()
        self.camera = camera
        
    def publish_image(self):
        # Cargar la imagen de archivo
        ## image_path = 'path/to/your/image.jpg'
        ## image = cv2.imread(image_path)
        image = self.camera.get_frame()

        # Convertir la imagen de OpenCV a ROS y publicarla
        if image is None: pass
        ros_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.image_pub.publish(ros_image)
        print("Publising")

    def run(self):
        print("Go publish")
        rate = rospy.Rate(5)  # Publicar imágenes a 10 Hz

        while not rospy.is_shutdown():
            try:
                self.publish_image()
                rospy.log("Publishing")
                print("Publishing")


            except: continue
            rate.sleep()

if __name__ == '__main__':
    try:
        camera = Camera_OAK()
        ## camera()
        ## image_publisher = ImagePublisher(camera)
        ## image_publisher.run()
        list_run = [camera, ImagePublisher(camera)]
        [item.start() for item in list_run]
        [item.join() for item in list_run]

    except rospy.ROSInterruptException:
        pass

