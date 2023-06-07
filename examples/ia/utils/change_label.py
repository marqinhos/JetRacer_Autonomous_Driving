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

class ChangeLabels:
    """Class to change YOLO *.txt labels 
    """

    def __init__(self, labels2change: dict = {0: 10}) -> None:
        """Funciton for init class

        Args:
            labels2change (dict(int), optional): Dictionary what the key is the label to change and the value is the new label. Defaults to {0: 10}.
        """
        self.root_path = "./data/labels/train/*.txt"
        self.labels2change = labels2change

    def __call__(self) -> None:

        for filename in glob.glob(self.root_path, recursive=True):
            with open(filename, 'r') as f:
                original_lines = f.readlines()
                output_line = ""
                for line in original_lines:
                    line_split = line.split(" ")
                    if int(line_split[0]) in self.labels2change.keys():
                        line_split[0] = self.labels2change[int(line_split[0])]
                        line = " ".join(line_split)
                    output_line += line
                #print(output_line)
                
                with open(filename, 'w') as out:
                    out.write(output_line)

if __name__ == "__main__":

    ChangeLabels()()