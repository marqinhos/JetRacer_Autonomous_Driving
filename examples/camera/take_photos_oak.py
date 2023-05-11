import cv2
import depthai as dai
import time
import os 
import numpy as np


class Camera_OAK:

    def __init__(self) -> None:

        self.width = 640
        self.height = 480
        self.time2save = 5
        self.__show = False
        self.root_folder = os.path.join(".", "data")


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


    def __call__(self) -> None:
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
                            now_time = time.time()

                            if round(abs(now_time - start_time), 1) == 5:
                                start_time = now_time
                                print("5 Segundo")
                                self.save_photo(frame, num_img)
                                num_img += 1
                            if self.__show:
                                cv2.putText(frame, "Fps: {:.2f}".format(fps), (10, 10), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255,255,255))
                                cv2.imshow(stream, frame)

                if self.__show:
                    if time.time() - tfps >= 1.0:
                        scale = time.time() - tfps
                        for stream in fpsCounter.keys():
                            lastFpsCount[stream] = fpsCounter[stream] / scale
                        fpsCounter = {}
                        tfps = time.time()

                    if cv2.waitKey(1) == ord('q'):
                        break

    
    def save_photo(self, frame: np.ndarray, num_frame: int) -> bool:
        """Function to save a photo in a directory

        Args:
            frame (np.ndarray): Image want to save
            num_frame (int): Number of frame, to save 

        Returns:
            bool: Return True if save could be do it else return False
        """
        filename = self.root_folder + "/image_" + str(num_frame) + ".jpg"
        try:
            cv2.imwrite(filename, frame)
            return True
        except: return False



if __name__ == "__main__":
    Camera_OAK()()

