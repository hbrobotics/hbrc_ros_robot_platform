#!/bin/bash
echo "================ Installing ROS 2 and the micro-ROS build system. ================"

if [[ "$#" != "1" ]]
then
    echo "usage: uros_install.sh WS_DIR"
    exit 1
fi

echo "================ Create entirely NEW workspace $WS_DIR. ================"
WS_DIR="$1"
if [[ -d "$WS_DIR" ]]
then
    echo "To stop removal of $WSDIR, type Control-C!"
    echo ""
    for count in 9 8 7 6 5 4 3 2 1
    do
	echo "Entirely remove $WS_DIR in $count second(s)"
	sleep 1
    done
    echo "==== Removing $WS_DIR directory ===="
    rm -rf "$WS_DIR"
fi
echo "==== Create brand new $WS_DIR directory ===="
mkdir -p "$WS_DIR"

# All commands are executed in $WS_DIR from here on out.
cd $WS_DIR

echo "==== Verify the ROS 2 installation. ===="
source /opt/ros/foxy/setup.bash

if [[ "$ROS_DISTRO" != "foxy" ]]
then
	echo "ROS2 is not is at the foxy release"
	exit 1
fi

if [[ ! "$(which ros2)" ]]
then
	echo "ros2 command is not present"
	exit 1
fi


echo "================ Download the micro-ROS tools ================"
mkdir src
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
echo "==== Update dependencies using rosdep ===="
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
echo "==== Install pip3  ===="
sudo apt-get install python3-pip -y
echo "==== Do colcon build ===="
colcon build
source install/local_setup.bash

echo "================ Create firmware step ================"
ros2 run micro_ros_setup create_firmware_ws.sh freertos nucleo_f767zi

echo "================ Configure firmware step ================"
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial

echo "================ Building the Firmware ================"
ros2 run micro_ros_setup build_firmware.sh
source /opt/ros/foxy/setup.bash
source install/local_setup.bash

echo "================ Creating the micro-ROS Agent ================"
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
