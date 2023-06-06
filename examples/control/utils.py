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


import numpy as np

######################
## A lo mejor se puede mirar de intentar predecir hacia donde va 

def point_middle(point1, point2):
    return (point1 + point2) // 2

def extract_file(data_file: np.ndarray):
    dict_4_corner = {}
    middle_line = {}
    cont = 0
    cont2 = 0

    for j in range(len(data_file)):
        row = data_file[j]
        in_line = []
        for i in range(len(row)):
            if row[i] == 1:
                is_corner = False
                try:
                    if row[i-1] == 0:
                        is_corner = True
                    elif row[i+1] == 0:
                        is_corner = True
                except:
                    if i == 0: is_corner = True
                    elif i == len(row): is_corner = True

                if is_corner:
                    dict_4_corner[cont] = [j, i]
                    in_line.append(i)
                    cont += 1
        if len(in_line) == 2:
            middle_line[cont2] = [j, point_middle(in_line[0], in_line[1])]
            cont2 += 2

    return dict_4_corner, middle_line


