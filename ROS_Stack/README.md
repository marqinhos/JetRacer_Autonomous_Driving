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

Them run "script.sh" and the jetracer is running all files and driving the circuit

## Software
in progress**
## Development

List TODO:
- [x] [JetRacer publish image OAK-1]()
- [x] [IA to segment lane and lines]()
- [x] [Brain to process the vel linear and angular]()
- [x] [Program to extract a desired point with segmentation]()
- [x] [Object front detection with RPlidar]()
- [ ] [Perform IA segmentation]()
- [ ] [Perform Extraction of desired point]()
- [ ] [Use Intel RealSense Camera]()




## License
**JetRacer Autonomous Driving** is available under next license:

* GPL-3.0 License: See [LICENSE](./LICENSE) file for details.
## Author:
(c) 2023 ([Marcos Fernández](https://github.com/marqinhos))

