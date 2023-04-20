import cv2
import numpy as np
from robot_class import JetRacer
# import onnx
# import onnx_tensorrt.backend as backend
from ultralytics import YOLO
from utils import extract_file

class BrainJetRacer:
    """Class with the logic to control JetRacer in a speedway

        Imputs:
            - Frame from JetRacer camera
        

    """

    def __init__(self, jetracer: JetRacer, frame, path_ia_model) -> None:
        self.jetRacer = jetracer
        self.frame = frame
        self.running = True
        # self.ia_model = onnx.load(path_ia_model)
        # self.engine = backend.prepare(model, device='CUDA:1')
        self.ia_model = YOLO(path_ia_model)

    @staticmethod
    def __wait4frame(frame):
        return False if frame is None else True
    
    def __stop(self):
        self.jetRacer.stop()
        self.running = False

    def update_frame(self, new_frame):
        self.frame = new_frame if new_frame is not None else self.frame

    def run(self):
        
        while self.running:
            if not self.__wait4frame(self.frame): continue

            ######### -Extract frame features- #########
            linear_vel, angular_vel = self.get_frame_features()

            ######### -JetRacer Run- #########
            self.jetRacer.run()

    def get_frame_features(self):
        # output_data = self.engine.run(self.frame)[0]
        result = self.ia_model.predict(source=self.frame)
        




















