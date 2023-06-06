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


import glob
import os
import cv2

class SaveIMG:

    def __init__(self) -> None:
        """Init function. 
            -> root_path_labels (str): Path that contain all labels
            -> root_path_images (str): Path that contain all images
            -> path2save_img (str): Path for save selective images
            -> img_extension (str): Extension images
        """
        self.root_path_labels = "./data/labels/*.txt"
        self.path2save_img = "./data/images/"
        self.img_extension = ".jpg"
        self.root_path_images = "./data/all_img/"


    def __call__(self):

        for filename in glob.glob(self.root_path_labels, recursive=True):

            pre, ext = os.path.splitext(filename)
            img_name = os.path.basename(pre)
            img = cv2.imread(self.root_path_images+img_name+ self.img_extension)
            cv2.imwrite(self.path2save_img+img_name+ self.img_extension, img)

            

if __name__ == "__main__":
    SaveIMG()()