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
        




















