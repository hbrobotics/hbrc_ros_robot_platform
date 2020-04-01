#!/bin/bash

# This file is intended to be run from other scripts vie a `source install_start.sh` command.
# It sets up all of the environment variables and makes darn sure that the
# hbrc_ros_robot_platform repository is up-to-date.  This piece of code can be safely
# executed multiple times.

# Some commonly used macros:
SUDO="sudo -H"
APT_ADD_REPOSITORY="sudo add-apt-repository --yes"
APT_INSTALL="$SUDO apt install --yes"
APT_UPDATE="$SUDO apt update"
BASHRC="$HOME/.bashrc"
PIP_INSTALL="pip3 install"
WORKON_HOME="$HOM/.virtualenvs"
MKVIRTUALENV="mkvirtualenv -p 3.6" 

# Capture the PROJECTS_HOME directory root:
PROJECT_HOME=`pwd | sed s,/hbrc_ros_robot_platform.*,,g`
HR2_HOME=$PROJECT_HOME/hbrc_ros_robot_platform

# Ensure that the repository is up to date.  The following URL was helpful:
#
#     https://stackoverflow.com/questions/3258243/check-if-pull-needed-in-git
#
# The first answer was not useful, but an answer a little further down proved to be workable:

# Force the remote database to be yanked over the wire:
git fetch

# Now figure out if we need to do a `git pull`:
LOCAL_HEAD=`git rev-parse HEAD`   # Get commit name for local branch
REMOTE_HEAD=`git rev-parse @{u}`  # Shear magic secret sauce!
# echo LOCAL_HEAD=$LOCAL_HEAD     # Debugging only
# echo REMOTE_HEAD=$REMOTE_HEAD   # Debugging only
if [ $LOCAL_HEAD != $REMOTE_HEAD ]
then
    echo "**************** Updating out-of-date hbrc_ros_robot_platform repository ..."
    git pull
else
    echo "The hbrc_ros_robot_platform repository is up-to-date."
fi

