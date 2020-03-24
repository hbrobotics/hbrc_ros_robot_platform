#!/bin/bash

# Refresh the repositiory:
echo "Refreshing the hbrc_ros_robot_platform repository ..."
git pull

# Now install the electrical stuff:
echo "Installing the electrical portion of the project ..."
source electrical/install_ee.sh

# Now install the mechancal stuff:
echo "Installing the mechanical porition of the project ..."
source mechanical/install_me.sh

# 
echo "Please type 'source ~/.bashrc' to enable python virtual environments."
echo "Next type 'workon hr2' to enter the 'hr2' virtual environment."
echo "To exit the hr2 python virtual environment, type 'deactivate'."
