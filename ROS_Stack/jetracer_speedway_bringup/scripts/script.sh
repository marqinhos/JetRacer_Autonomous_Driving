#!/bin/bash

# Función que se ejecuta al recibir la señal SIGINT
function stop_processes {
    echo "Kill process..."
    # Matar procesos de ROS
    pkill -f "publish_goalPoint.py"

    sshpass -p "$JETSON_PASSWORD" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $JETSON_USER@$JETSON_IP "
    pkill -f "camera_publish.py";
    pkill -f "jetracer_brain.py";
    pkill -f "jetracer_driver.py";
    rosnode kill /rplidarNode;
    pkill -f "roslaunch rplidar_ros rplidar.launch""

    pkill -f "security_stop.py"
    pkill -f "roscore"
    kill $sshpass_pid_1
    exit 1
}

######################################################################

# Instalar sshpass si no está instalado
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

######################################################################
# Cargar las variables de entorno de .env con direnv
eval "$(direnv export bash)"

direnv allow
sshpass_pid_1=""

source ~/catkin_ws/devel/setup.bash;
roscore &

# Asociar la función stop_processes a la señal SIGINT
trap stop_processes SIGINT

# Bucle para intentar conexión mediante sshpass
while true; do
  sshpass -p "$JETSON_PASSWORD" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $JETSON_USER@$JETSON_IP "exit"
  if [ $? -eq 0 ]; then
    break # Si la conexión se establece, salir del bucle
  fi
  sleep 1 # Esperar un segundo antes de volver a intentar la conexión
done


################################### IMAGE ###################################
# Connect to Jetson Nano
sshpass -p "$JETSON_PASSWORD" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $JETSON_USER@$JETSON_IP "
cd ~/catkin_ws; 
source ~/catkin_ws/devel/setup.bash;
export ROS_IP=$JETSON_IP && export ROS_MASTER_URI=http://$USER_IP:11311;
cd src;
sudo chmod 666 /dev/ttyUSB0;
roslaunch rplidar_ros rplidar.launch &
if [ -d "ros_jetracer_control" ]; then
    rm -rf ros_jetracer_control
fi
git clone https://github.com/marqinhos/ros_jetracer_control;
cd ..;
catkin_make;
cd src;
cd ros_jetracer_control/ROS_Stack/jetracer_speedway_sensors/src;
chmod +x camera_publish.py;
cd ../..;
cd jetracer_speedway_control/src;
chmod +x jetracer_brain.py;
cd ../..;
cd jetracer_speedway_drivers/src;
chmod +x jetracer_driver.py;
cd ../..;

# cd jetracer_speedway_bringup/src;
roslaunch jetracer_speedway_bringup run_jetson.launch" &

################################### PID ###################################
## Storage pid for sshpass
sshpass_pid_1=$! 


cd ~/catkin_ws; 
source ~/catkin_ws/devel/setup.bash;
catkin_make;
export ROS_IP=$USER_IP;
cd src/ROS_Stack/jetracer_speedway_navigation/src;
chmod +r models/best.pt;
chmod +x publish_goalPoint.py;
cd ../..;
cd jetracer_speedway_bringup/src;
roslaunch jetracer_speedway_bringup run_master.launch

echo 'Se esta ejecutando'