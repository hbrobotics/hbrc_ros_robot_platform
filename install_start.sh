#!/bin/bash

# This file is intended to be run from other scripts via a `source install_start.sh` command.
# It sets up all of the environment variables and makes darn sure that the
# hbrc_ros_robot_platform repository is up-to-date.  This piece of code can be safely
# executed multiple times.
#
# This script defines some commonly used shell variables *AND* makes sure that the master
# branch is up to date.  See below about update the master branch.

# Some commonly used macros:
SUDO="sudo -H"
APT_ADD_REPOSITORY="sudo add-apt-repository --yes"
APT_INSTALL="$SUDO apt install --yes"
APT_UPDATE="$SUDO apt update"
BASHRC="$HOME/.bashrc"
PIP_INSTALL="pip3 install"
WORKON_HOME="$HOME/.virtualenvs"
MKVIRTUALENV="mkvirtualenv -p 3.6" 

# Capture the PROJECTS_HOME directory root:
PROJECT_HOME=`pwd | sed s,/hbrc_ros_robot_platform.*,,g`
HR2_HOME=$PROJECT_HOME/hbrc_ros_robot_platform

# The master branch is configured to be "read-only in the sense that it is *NEVER* modified on
# the local machine.  Modifications on other branches are allowed, but not to the local master.
# What this means is that it is always safe to do a `git pull` into the master branch.  We do
# over think this, we just attempt to checkout master and do the pull.  If it fails, we let
# you know that the checkout failed:
echo "Attempting to ensure that master branch is checked out ..."
if git checkout master
then
    echo "**************** Switched to master branch and updating it ..."
    git pull
else
    echo "!!!!!!!!!!!!!!!! Unable to update master branch; no master branch update occurred."
fi

