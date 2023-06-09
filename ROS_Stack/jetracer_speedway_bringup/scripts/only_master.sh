#!/bin/bash

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


## Function to kill and clean all JetRaces Autonomous Driving process
function stop_processes {
    echo "Kill process..."
    ## Kill all ROS process
    pkill -f "publish_goalPoint.py"
    pkill -f "object_detection.py"
    pkill -f "security_stop.py"
    pkill -f "roscore"
    kill $sshpass_pid_1
    exit 1
}

# Verify direnv
if ! command -v direnv &> /dev/null
then
    echo "direnv not installed. Instaling......"
    sudo apt-get update && sudo apt-get install -y direnv
else
    echo "direnv already installed"
fi

direnv allow
######################################################################
## Load enviorement var with direnv
eval "$(direnv export bash)"

direnv allow
sshpass_pid_1=""
## Add stop_processes function to SINGINT signal
trap stop_processes SIGINT

cd ~/TFG/tfg_ws; 
source devel/setup.bash;
catkin_make;
export ROS_IP=$USER_IP;
cd src/JetRacer_Autonomous_Driving/ROS_Stack;
roslaunch jetracer_speedway_bringup run_master.launch