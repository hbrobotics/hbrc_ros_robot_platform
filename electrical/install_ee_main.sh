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

# Let user know install_ee_main.sh is running:
echo "**************** Starting body of install_ee_main.sh ..."

# Capture the PROJECTS_HOME directory root:
PROJECT_HOME=`pwd | sed s,/hbrc_ros_robot_platform.*,,g`
HR2_HOME=$PROJECT_HOME/hbrc_ros_robot_platform

# Make sure all of the common stuff has already been done:
source $HR2_HOME/install_start.sh

# Install the common stuff:(
(cd $HR2_HOME ; ./install_common.sh)

# The virtual envirionment stuff is brittle, so we have to source the file directly:
# Append "source /usr/local/bin/virtualenvwrapper.sh" to ~/.bashrc .
VIRTUALENVWRAPPER_SH=`which virtualenvwrapper.sh`
if [ -z "$VIRTUALENVWRAPPER_SH" ]
then
    echo "???????????????? VIRTUALENVWRAPPER_SH is empty"
else
    source $VIRTUALENVWRAPPER_SH
fi

# For debugging, please turn on the debugging flags below:
# set -x           # Trace execution.
# set -e           # Exit immediately on error result.
# set -o pipefail  # Fail if any commands in a pipeline fail.
# set -u           # Treat unset variables as an error.

# Install kicad:
if [ -z `which kicad` ]
then
    echo "**************** Installing KiCad package ..."
    echo "Add KiCad to your package repository database:"
    $SUDO add-apt-repository --yes ppa:js-reynaud/kicad-5.1
    echo "Update your package repository database:"
    $APT_UPDATE
    echo "Install kicad. This over a gigabyte of code/libraires to download ..."
    $APT_INSTALL --install-recommends kicad
    echo "Install KiCad demo projects:"
    $APT_INSTALL kicad-demo
else
    echo "Kicad previously installed"
fi

# Clone bom_manager:
if [ ! -d $PROJECT_HOME/bom_manager ]
then
    echo "**************** Cloning bom_manager repository into $PROJECT_HOME ."
    echo PROJECT_HOME=$PROJECT_HOME
    (cd $PROJECT_HOME ; pwd ;git clone https://github.com/waynegramlich/bom_manager.git)
else
    echo "Making sure that bom_manager is up to date ..."
    (cd $PROJECT_HOME/bom_manager ; git pull)
fi

# Clone kicube32:
if [ ! -d $PROJECT_HOME/kicube32 ]
then
    echo PROJECT_HOME=$PROJECT_HOME
    echo "**************** Cloning kicube32 repository into $PROJECT_HOME ."
    (cd $PROJECT_HOME ; pwd ; git clone https://github.com/waynegramlich/kicube32.git)
else
    echo "Making sure that kicube32 is up to date ..."
    (cd $PROJECT_HOME/kicube32 ; git pull)
fi

# The virtual wrapper script breaks under debugging, so turn everything off:
set +x           # Trace execution.
set +e           # Exit immediately on error result.
set +o pipefail  # Fail if any commands in a pipeline fail.
set +u           # Treat unset variables as an error.

# Install bom_manager into hr2 env


# Install kipart into hr2 environment:
(workon hr2
 if [ -z `which kipart` ]
 then
    echo "**************** Installing kipart into hr2 virtual environment..."
    pip3 install kipart
 else
    echo "kipart previously installed into hr2 virtual environment."
 fi
 deactivate)

# Install kicube32 into hr2 environment:
echo "Installing kicube32 into hr2 virtual environment."
(workon hr2
 if [ -z `which kicube32` ]
 then
     echo "**************** Installing kicube32 into hr2 virtual environment..."
     $PIP_INSTALL $PROJECT_HOME/kicube32
 else
     echo "kicube32 prevously installed into hr2 virtual environment..."
     
 fi
 deactivate)

# Remind people to update their virtual environments:
echo "Please type 'source ~/.bashrc' to enable python virtual environments."
echo "Next type 'workon hr2' to enter the 'hr2' virtual environment."
echo "To exit the hr2 python virtual environment, type 'deactivate'."
