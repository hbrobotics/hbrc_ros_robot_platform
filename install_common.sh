#!/bin/bash

# Shell scripts are notorously difficult to read and debug.
# To aid in reading/debugging this script, there are deliberately plenty of comments.

# For debugging, please turn on the debugging flags below:
# set -x           # Trace execution.
# set -e           # Exit immediately on error result.
# set -o pipefail  # Fail if any commands in a pipeline fail.
# set -u           # Treat unset variables as an error.

# Set up the macros needed for everything else.
source install_start.sh

# Inform the user that packages are being installed and 
echo "Installing additional packages."
echo "TO INSTALL PACKAGES, YOU MAY BE PROMPTED FOR YOUR ROOT PASSWORD..."

# Make sure we have the build-essential package installed.
if [ -z `which make` ]
then
    echo "**************** Installing build-essential package ..."
    $APT_INSTALL build-essential
else
    echo "build-essential previously installed"
fi

# Make sure we have git version management system installed.
if [ -z `which git` ]
then
    echo "**************** Installing git version management system."
    $APT_INSTALL git
else
    echo "git version management system previously installed."
fi

# Make sure we have vim editor installed.
if [ -z `which vim` ]
then
    echo "**************** Installing vim editor ..."
    $APT_INSTALL vim
else
    echo "vim editor previously installed"
fi

# Make sure we have emacs editor installed.
if [ -z `which emacs` ]
then
    echo "**************** Installing emacs editor ..."
    $APT_INSTALL emacs
else
    echo "emacs editor previously installed"
fi

# Force Python3 to be the default Python.  Python2 is the default Python until
# Ubuntu 20.04 gets released.  We compute PYTHON_REALPATH and PYTHON3_REALPATH
# and if they are not equal, we force an update.
PYTHON_PATH=`which python`
PYTHON3_PATH=`which python3`
PYTHON_REALPATH=`realpath $PYTHON_PATH`
PYTHON3_REALPATH=`realpath $PYTHON3_PATH`
if [ "$PYTHON_REALPATH" != "$PYTHON3_REALPATH" ]
then
    echo "**************** Forcing Python3 to be the default Python ..."
    sudo update-alternatives --install /usr/bin/python python $PYTHON3_REALPATH 2
else
    echo "Python3 is already the default Python."
fi

# Install python3_pip3:
if [ -z `which pip3` ]
then
    echo "**************** Installing Python pip3"
    $APT_INSTALL python3-pip
else
    echo "Python pip3 previously installed."
fi

# Globally install Python virtual environments:
if [ -z `which virtualenvwrapper.sh` ]
then
    echo "**************** Globally Installing Python virtual environments ..."
    $SUDO $PIP_INSTALL virtualenvwrapper
else
    echo "Python virtual environments previously installed"
fi

# Modify ~/.bashrc to support virtual environments by appending the following to ~/.bashrc :
#        export VIRTUALENVWRAPER_PYTHON=/user/bin/python3
#        export WORKON_HOME=$HOME/.virtualenvs
#        export PROJECT_HOME=...  # Root projects directory
#        export VIRTUALENVWRAPER_WORKON_CD=1
#        source /usr/local/bin/virtualenvwrapper.sh


# Append "export VIRTUALENVWRAPER_PYTHON=/user/bin/python3" to ~/.bashrc .
if [ -z "`grep VIRTUALENVWRAPER_PYTHON $BASHRC`" ]
then
    echo "**************** Add VIRTUALENVWRAPER_PYTHON to $BASHRC ..."
    echo "export VIRTUALENVWRAPER_PYTHON=`which python3`" >> $BASHRC
else
    echo "VIRTUALENVWRAPER_PYTHON already in $BASHRC ."
fi

# Append "export WORKON_HOME=$HOME/.virtualenvs" to ~/.bashrc .
if [ -z "`grep WORKON_HOME $BASHRC`" ]
then
    echo "**************** Add WORKON_HOME to $BASHRC ..."
    echo "export WORKON_HOME=$WORKON_HOME" >> $BASHRC
else
    echo "WORKON_HOME is already in $BASHRC ."
fi

# Append "**************** export PROJECT_HOME=... " to ~/.bashrc .
if [ -z "`grep PROJECT_HOME $BASHRC`" ]
then
    echo "**************** Add PROJECT_HOME to $BASHRC ..."
    echo "export PROJECT_HOME=$PROJECT_HOME" >> $BASHRC
else
    echo "PROJECT_HOME is already in $BASHRC ."
fi

# Append "export VIRTUALENVWRAPER_WORKON_CD=1" to ~/.bashrc .
if [ -z "`grep VIRTUALENVWRAPER_WORKON_CD $BASHRC`" ]
then
    echo "**************** Add VIRTUALENVWRAPER_WORKON_CD to $BASHRC ..."
    echo "export VIRTUALENVWRAPER_WORKON_CD=1" >> $BASHRC
else
    echo "VIRTUALENVWRAPER_WORKON_CD already in $BASHRC ."
fi

# Append "source /usr/local/bin/virtualenvwrapper.sh" to ~/.bashrc .
VIRTUALENVWRAPPER_SH=`which virtualenvwrapper.sh`
if [ -z "`grep virtualenvwrapper.sh $BASHRC`" ]
then
    echo "**************** Add 'source .../virtualenvwrapper.sh' to $BASHRC ..."
    echo "source $VIRTUALENVWRAPPER_SH" >> $BASHRC
else
    echo "source .../virtualenvwrapper.sh already in $BASHRC ."
fi

# The virtual wrapper script breaks under debugging, so turn all debugging off.
set +x           # Trace execution.
set +e           # Exit immediately on error result.
set +o pipefail  # Fail if any commands in a pipeline fail.
set +u           # Treat unset variables as an error.

# For some weird reason, doing a "source $BASHRC" does not properly get
# the virtual environment wrapper variables set properly.  So, instead
# we set them all manually below:
export VIRTUALENVWRAPER_PYTHON=`which python3`
export WORKON_HOME=$WORKON_HOME
export PROJECT_HOME=$PROJECT_HOME
export VIRTUALENVWRAPER_WORKON_CD=1
source `which virtualenvwrapper.sh`

# Create the "hr2" Python virtual environment:
if [ ! -d $WORKON_HOME/hr2 ]
then
    echo "**************** Create hr2 python virtual environment ..."
    $MKVIRTUALENV -a $PROJECT_HOME/hbrc_ros_robot_platform hr2
else
    echo "hr2 Python virtual environment already exists."
fi

