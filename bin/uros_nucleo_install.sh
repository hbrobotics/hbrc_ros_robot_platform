#!/bin/bash

# This file is licensed using the "MIT License" below:
#
####################################################################################################
#
# MIT License
#
# Copyright 2021 Home Brew Robotics Club
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following
# conditions:
# 
# The above copyright notice and this permission notice shall be included in all copies
# or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
####################################################################################################


echo "================ Installing ROS 2 and the micro-ROS build system. ================"

if [[ "$#" != "1" ]]
then
    echo "usage: uros_install.sh WS_DIR"
    exit 1
fi

# There are steps below that do no work inside of a python virtual environment.
# We detect this and ask the user to disable their Python Virtual Environment.
if [[ "$VIRTUAL_ENV" ]]
then
    echo "This script will not work with a Python virtual environment enabled."
    echo "A Python Virtual Environment is currently enabled."
    echo "Please type 'deactivate' to disable the Python Virtual Environment."
    echo "After the script is done remember to renable the Python Virutal Environment by typing:"
    echo "    workon $(echo $VIRTUAL_ENV | sed 's,^.*/,,g')"
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

echo "================ Some final notes: ================"
echo "To rebuild the the microcontroller firmware from scatch (similar to 'make clean'):"
echo "    cd $WS_DIR"
echo "    source /opt/ros/foxy/setup.bash"
echo "    source install/local_setup.bash"
echo "    ros2 run micro_ros_setup build_firmware.sh"
echo ""
echo "To incrementally rebuild the microcontroller firmware (similar to `make`):"
echo "    cd $WS_DIR"
echo "    source /opt/ros/foxy/setup.bash"
echo "    source install/local_setup.bash"
echo "    ros2 run micro_ros_setup build_firmware.sh -f"
echo ""
echo "To flash the firmware to the microcontroller run the following commands:"
echo "    cd $WS_DIR"
echo "    source /opt/ros/foxy/setup.bash"
echo "    source install/local_setup.bash"
echo "    ros2 run micro_ros_setup flash_firmware."
echo ""
