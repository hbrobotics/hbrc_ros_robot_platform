#!/bin/bash

# This file is licensed using the "MIT License" below:
#
####################################################################################################
#
# MIT License
#
# Copyright 2020 Home Brew Robotics Club
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

# Shell scripts are notorously difficult to read and debug.
# To aid in reading/debugging this script, there are deliberately plenty of comments.

# Let user know install_me_main.sh is running:
echo "**************** Starting body of install_me_main.sh ..."

# Make sure all of the common stuff has already been done:
# Capture the PROJECTS_HOME directory root:
PROJECT_HOME=`pwd | sed s,/hbrc_ros_robot_platform.*,,g`
HR2_HOME=$PROJECT_HOME/hbrc_ros_robot_platform

# Set up the enviroment variables:
source $HR2_HOME/install_start.sh

# The virtual envirionment stuff is brittle, so we have to source the file directly:
# Append "source /usr/local/bin/virtualenvwrapper.sh" to ~/.bashrc .
VIRTUALENVWRAPPER_SH=`which virtualenvwrapper.sh`
if [ -z "$VIRTUALENVWRAPPER_SH" ]
then
    echo "???????????????? VIRTUALENVWRAPPER_SH is empty"
else
    source $VIRTUALENVWRAPPER_SH
fi
# Install the common stuff:
(cd $HR2_HOME ; ./install_common.sh)

# For debugging, please turn on the debugging flags below:
# set -x           # Trace execution.
# set -e           # Exit immediately on error result.
# set -o pipefail  # Fail if any commands in a pipeline fail.
# set -u           # Treat unset variables as an error.

# Make sure we have openscad installed:
if [ -z "`which openscad`" ]
then
    # While there is no standard openscad package for 18.04 or 18.10, there is one for 19.10.
    # We side step the issue by just grabbing openscad directly from the openscad repository.
    echo "**************** Installing openscad package ..."
    $APT_ADD_REPOSITORY ppa:openscad/releases
    $APT_UPDATE
    $APT_INSTALL openscad
else
    echo "openscad previously installed."
fi
    
# Make sure markdown to HTML convert is installed:
if [ -z "`which markdown`" ]
then
    echo "**************** Installing markdown to HTML converter ..."
    $APT_INSTALL markdown
else
    echo "markdown to HTML converter previously installed."
fi

# Install qcad:
if [ -z "`which qcad`" ]
then
    echo "**************** Installing qcad package ..."
    $APT_ADD_REPOSITORY ppa:alex-p/qcad
    $APT_UPDATE
    $APT_INSTALL qcad
else
    echo "qcad previously installed"
fi

# Any time we mess with Pthon virtual environments, we need to disable debugging:
set +x           # Trace execution.
set +e           # Exit immediately on error result.
set +o pipefail  # Fail if any commands in a pipeline fail.
set +u           # Treat unset variables as an error.

# Install Python mypi static type checker:
if [ -z "`which mypy`" ]
then
    echo "Installing Python mypy static type checker ..."
    (workon hr2; $PIP_INSTALL mypy ; deactivate)
else
    echo "Python mypy static type checker previously installed."
fi

# Install Python flake8 code style checker:
if [ -z "`which flake8`" ]
then
    echo "**************** Installing Python flake8 code style checker ..."
    (workon hr2; $PIP_INSTALL flake8 ; deactivate)
else
    echo "Python flake8 code style checker previously installed."
fi

# Install Python pydocstyle documentation style checker:
if [ -z "`which pydocstyle`" ]
then
    echo "**************** Installing Python pydocstyle documentation style checker ..."
    (workon hr2; $PIP_INSTALL pydocstyle ; deactivate)
else
    echo "Python pydocstyle documentation style checker previously installed."
fi

# Install pytest testing system:
if [ -z "`which pytest`" ]
then
    echo "**************** Installing Python testing system ..."
    (workon hr2; $PIP_INSTALL pytest ; deactivate)
else
    echo "Python testing system previously installed."
fi

# Remind people to update their virtual environments:
echo "Please type 'source ~/.bashrc' to enable python virtual environments."
echo "Next type 'workon hr2' to enter the 'hr2' virtual environment."
echo "To exit the hr2 python virtual environment, type 'deactivate'."


