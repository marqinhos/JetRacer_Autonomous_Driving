# JetRacer Autonomous Driving 

[![MKT](https://shields.io/badge/license-Copyleft-red.svg)](./LICENSE)
[![MKT](https://shields.io/badge/version-v1.0.0-blue.svg)]()
[![MKT](https://shields.io/badge/language-Python3-r.svg?logo=python)](https://www.python.org/)
[![MKT](https://shields.io/badge/plataform-ROS-lightblue.svg?logo=ROS)](https://www.ros.org/)
[![MKT](https://shields.io/badge/github-gray.svg?logo=github)](https://github.com/marqinhos?tab=repositories)


## Description

Use next packages for own project, using the base name "jetracer_speedway":
- jetracer_speedway
- jetracer_speedway_bringup
- jetracer_speedway_msg
- jetracer_speedway_sensors
- jetracer_speedway_driver
- jetracer_speedway_control
- jetracer_speedway_navigate

Download package in your main computer. Go to jetracer_speedway_bringup/scripts and create a .envrc file, next writte the following information:
- export JETSON_IP=main_pc_ip
- export JETSON_PASSWORD=password_jetson
- export JETSON_USER=name_user_jetson
- export USER_IP=jetson_ip


## Software

#### To run all project:
Go to scripts folder into jetracer_speedway_bringup package. Them run script.sh.
  ``` bash
  ./script.sh
  ```

#### To run only a ros package:
Go to launchs folder into jetracer_speedway_bringup package. Them create a new .launch file, as sample:
``` html
<!-- version: 1.0.0 -->

<!--
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
-->

<launch>
  <!-- Set config_file path -->
  <rosparam file="$(find jetracer_speedway_bringup)/config/config.yaml" command="load" />
  <param name="config_file" value="$(find jetracer_speedway_bringup)/config/config.yaml"/>

  <!-- Run node into package -->
  <node pkg="name_package" type="name_program" name="name_node" />

</launch>
```
Where:
- name_package: it is the name of the package that contains the ROS node to be executed.
- name_program: it is the name of the python program that run node.
- name_node: it is the name of the node to be execute.

For run this .launch file:
``` bash
  roslaunch jetracer_speedway_bringup name_launch.launch
```
Where:
- name_launch: it is the name of the pass launch file created.

## Development

List TODO:
- [x] [JetRacer publish image OAK-1]()
- [x] [JetRacer publish image Intel RealSense]()
- [x] [IA to segment objects in driving area]()
- [x] [IA to segment driving area]()
- [x] [Program to extract a objetive point with segmentation]()
- [x] [Brain to process the objetive point to convert in linear velocity and turn angle to the JetRacer]()
- [x] [Object front detection with RPlidar]()
- [x] [Perform IA segmentation]()
- [x] [Perform navigattion]()
- [ ] [Perform Extraction of desired point]()


## License
**JetRacer Autonomous Driving** is available under next license:

* GPL-3.0 License: See [LICENSE](./LICENSE) file for details.
## Author:
(c) 2023 ([Marcos Fernández](https://github.com/marqinhos))

