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

    sshpass -p "$JETSON_PASSWORD" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $JETSON_USER@$JETSON_IP "
    pkill -f "camera_intel_publish.py";
    pkill -f "jetracer_brain.py";
    pkill -f "jetracer_driver.py";
    rosnode kill /rplidarNode;
    pkill -f "roslaunch rplidar_ros rplidar_a1.launch""

    pkill -f "security_stop.py"
    pkill -f "roscore"
    kill $sshpass_pid_1
    exit 1
}

######################################################################

## Try to install sshpass
if ! command -v sshpass &> /dev/null
then
    echo "sshpass no está instalado. Instalando..."
    sudo apt-get update && sudo apt-get install -y sshpass
else
    echo "sshpass ya está instalado"
fi

# Verificar si direnv está instalado y, en caso contrario, instalarlo
if ! command -v direnv &> /dev/null
then
    echo "direnv no está instalado. Instalando..."
    sudo apt-get update && sudo apt-get install -y direnv
else
    echo "direnv ya está instalado"
fi
direnv allow
######################################################################
## Load enviorement var with direnv
eval "$(direnv export bash)"

direnv allow
sshpass_pid_1=""

source ~/catkin_ws/devel/setup.bash;
roscore &

## Add stop_processes function to SINGINT signal
trap stop_processes SIGINT

## Connect to Jetson Nano via sshpass
while true; do
  sshpass -p "$JETSON_PASSWORD" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $JETSON_USER@$JETSON_IP "exit"
  if [ $? -eq 0 ]; then
    break 
  fi
  sleep 1 
done

################################### IMAGE ###################################
## Run in Jetson Nano
sshpass -p "$JETSON_PASSWORD" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $JETSON_USER@$JETSON_IP "
cd ~/catkin_ws/src;
source ~/catkin_ws/devel/setup.bash;
if [ -d "jetracer_ws" ]; then
    echo "Workspace found";
else
  mkdir -p "jetracer_ws/src";
  cd "jetracer_ws";
  catkin_make;
  cd ..;
  echo "Jetracer workspace create";
fi
cd jetracer_ws;
echo $ROS_PACKAGE_PATH >> ~/.bashrc;
source devel/setup.bash;
export ROS_IP=$JETSON_IP && export ROS_MASTER_URI=http://$USER_IP:11311;
cd src;
catkin_init_workspace;
if [ -d "rplidar_ros" ]; then
    rm -rf rplidar_ros
fi
git clone https://github.com/Slamtec/rplidar_ros;
cd ..;
catkin_make;
cd src;
cd rplidar_ros/launch;
chmod +x rplidar_a1.launch;
cd ../..;
sudo chmod 666 /dev/ttyUSB0;
cd ..;
catkin_make;
cd src;
roslaunch rplidar_ros rplidar_a1.launch &
if [ -d "JetRacer_Autonomous_Driving" ]; then
    rm -rf JetRacer_Autonomous_Driving
fi
git clone https://github.com/marqinhos/JetRacer_Autonomous_Driving;
cd ..;
catkin_make;
cd src/JetRacer_Autonomous_Driving/ROS_Stack;
roslaunch jetracer_speedway_bringup run_jetson.launch" &


################################### PID ###################################
## Storage pid for sshpass
sshpass_pid_1=$! 


cd ~/TFG/tfg_ws; 
echo $ROS_PACKAGE_PATH >> ~/.bashrc;
source devel/setup.bash;
catkin_make;
export ROS_IP=$USER_IP;
#cd src;
#git clone https://github.com/marqinhos/JetRacer_Autonomous_Driving;
#cd ..;
#catkin_make;
cd src/JetRacer_Autonomous_Driving/ROS_Stack;
roslaunch jetracer_speedway_bringup run_master.launch

echo 'Se esta ejecutando'