#!/bin/bash

# Capture the PROJECTS_HOME directory root:
PROJECT_HOME=`pwd | sed s,/hbrc_ros_robot_platform.*,,g`
HR2_HOME=$PROJECT_HOME/hbrc_ros_robot_platform

# Force the repository to be up-to-date:
source $HR2_HOME/install_start.sh

# Install the mechanical specific installations:
(cd $HR2_HOME/mechanical ; ./install_me_main.sh)
