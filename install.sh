#!/bin/bash

# Shell scripts are notorously difficult to read and debug.
# To aid in reading/debugging this script, there are deliberately plenty of comments.

# For debugging, please turn on the debugging flags below:
set -x           # Trace execution.
set -e           # Exit immediately on error result.
set -o pipefail  # Fail if any commands in a pipeline fail.
set -u           # Treat unset variables as an error.

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
PROJECT_HOME=`(cd .. ; pwd)`

# Inform the user that packages are being installed and 
echo "Installing additional packages.  You may be prompted for your root password ..."

# Make sure we have the build-essential package installed.
if [ -z `which make` ]
then
    echo "Installing build-essential package ..."
    $APT_INSTALL build-essential
else
    echo "build-essential previously installed"
fi

# Make sure we have openscad installed:
if [ -z `which openscad` ]
then
    # While there is no standard openscad package for 18.04 or 18.10, there is one for 19.10.
    # We side step the issue by just grabbing openscad directly from the openscad repository.
    echo "Installing openscad package ..."
    $APT_ADD_REPOSITORY ppa:openscad/releases
    $APT_UPDATE
    $APT_INSTALL openscad
else
        echo "openscad previously installed"
fi
    
# Install python3:
if [ -z `which python3` ]
then
    echo "Installing Python 3 ..."
    $APT_INSTALL python3
else
    echo "Python3 previously installed"
fi

# Force Python3 to be the default Python.  Python2 is the default Python until
# Ubuntu 20.04 gets released.  We compute PYTHON_REALPATH and PYTHON3_REALPATH
# and if they are not equal, we force an update:
PYTHON_PATH=`which python`
PYTHON3_PATH=`which python3`
PYTHON_REALPATH=`realpath $PYTHON_PATH`
PYTHON3_REALPATH=`realpath $PYTHON3_PATH`
if [ "$PYTHON_REALPATH" -ne "$PYTHON3_REALPATH" ]
then
    echo "Forcing Python3 to be the default Python ..."
    sudo update-alternatives --install /usr/bin/python python $PYTHON3_REALPATH 2
else
    echo "Python3 is already the default Python"
fi

# Install python3_pip3:
if [ -z `which pip3` ]
then
    echo "Installing pip3"
    $APT_INSTALL python3-pip
else
    echo "pip3 previously installed"
fi

# There is a bug in the virtualenvwrapper.sh where it will choke if ZSH_VERSION
# is not empty:
export ZSH_VERSION=""

# Install Python virtual environments:
if [ ! -n `which virtualenvwrapper.sh` ]
then
    echo "Installing Python virtual environments ..."
    sudo $PIP_INSTALL virtualenvwrapper
else
    echo "Python virtual environments previously installed"
fi

# Install qcad:
if [ -z `which qcad` ]
then
    echo "Installing qcad package ..."
    $APT_ADD_REPOSITORY ppa:alex-p/qcad
    $APT_UPDATE
    $APT_INSTALL qcad
else
    echo "qcad previously installed"
fi

# Install kicad:
if [ -z `which kicad` ]
then
    echo "Installing kicad electronic computer aided design package ..."
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
    echo "Cloning bom_manager ..."
    (cd $PROJECT_HOME ; git clone https://github.com/waynegramlich/bom_manager.git)
else
    echo "bom_manager previously cloned"
fi

# Clone kicube32:
if [ ! -e $PROJECT_HOME/kicube32 ]
then
    echo "Cloning kicube32"
    (cd $PROJECT_HOME ; git clone https://github.com/waynegramlich/kicube32.git)
else
    echo "kicube32 previously cloned"
fi

# Globally install `virtualenvwrapper`:
if [ -z `which virtualenvwrapper.sh` ]
then
    echo "Install virtualenvwrapper ..."
    $SUDO $PIP_INSTALL virtualenvwrapper
else
    echo "vituealenvwrapper previously installed"
fi

# Modify ~/.bashrc to support virtual environments of the form:
#        export VIRTUALENVWRAPER_PYTHON=/user/bin/python3
#        export WORKON_HOME=$HOME/.virtualenvs
#        export PROJECT_HOME=...  # Root projects directory
#        export VIRTUALENVWRAPER_WORKON_CD=1
#        source /usr/local/bin/virtualenvwrapper.sh


# export VIRTUALENVWRAPER_PYTHON=/user/bin/python3
PATTERN=`grep VIRTUALENVWRAPPER_PYTHON $BASHRC`
if [ -z "$PATTERN" ]
then
    echo "Add VIRTUALENVWRAPPER_PYTHON to $BASHRC"
    echo "export VIRTUALENVWRAPPER_PYTHON=`which python3`" >> $BASHRC
else
    echo "VIRTUALENVWRAPPER_PYTHON already in $BASHRC"
fi

# export WORKON_HOME=$HOME/.virtualenvs
PATTERN=`grep WORKON_HOME $BASHRC`
if [ -z "$PATTERN" ]
then
    echo "Add WORKON_HOME to $BASHRC"
    echo "export WORKON_HOME=$WORKON_HOME" >> $BASHRC
else
    echo "WORKON_HOME is already in $BASHRC"
fi

#        export PROJECT_HOME=...  # Root projects directory
PATTERN=`grep PROJECT_HOME $BASHRC`
if [ -z "$PATTERN" ]
then
    echo "Add PROJECT_HOME to $BASHRC"
    echo "export PROJECT_HOME=$PROJECT_HOME" >> $BASHRC
else
    echo "PROJECT_HOME is already in $BASHRC"
fi

#        export VIRTUALENVWRAPER_WORKON_CD=1
PATTERN=`grep VIRTUALENVWRAPER_WORKON_CD $BASHRC`
if [ -z "$PATTERN" ]
then
    echo "Add VIRTUALENVWRAPER_WORKON_CD to $BASHRC"
    echo "export VIRTUALENVWRAPER_WORKON_CD=1" >> $BASHRC
else
    echo "VIRTUALENVWRAPER_WORKON_CD already in $BASHRC"
fi

#        source /usr/local/bin/virtualenvwrapper.sh
VIRTUALENVWRAPPER=`which virtualenvwrapper.sh`
PATTERN=`grep $VIRTUALENVWRAPPER $BASHRC`
if [ -z "$PATTERN" ]
then
    echo "Add 'source .../virtualenvwrapper.sh' to $BASHRC"
    echo "source `which virtualenvwrapper.sh`" >> $BASHRC
else
    echo "source .../virtualenvwrapper.sh already in $BASHRC"
fi

# Now slurp in all of the values so we can use them to set up some virtual environments:
source $BASHRC
export VIRTUALENVWRAPER_PYTHON=`which python3`
export WORKON_HOME=$WORKON_HOME
export PROJECT_HOME=$PROJECT_HOME
XXXX=`which virtualenvwrapper.sh`
source "$XXXX"
# mkvirtualenv --help
echo "================================================================"

# Set up virtual environments:
if [ ! -d $WORKON_HOME/hr2 ]
then
    echo "Create hr2 python virtual environment"
    $MKVIRTUALENV -a $PROJECT_HOME/hbrc_ros_robot_platform hr2
else
    echo "hr2 python virtual environment already exists"
fi
if [ ! -d $WORKON_HOME/bom_manager ]
then
    echo "Create hr2 python virtual environment"
    $MKVIRTUALENV -a $PROJECT_HOME/bom_manager bom_manager
else
    echo "bom_manager python virtual environment already exists"
fi
if [ ! -d $WORKON_HOME/kicube32 ]
then
    echo "Create kicube32 python virtual environment"
    $MKVIRTUALENV -a $WORKON_HOME/kicube32 kicube32
else
    echo "kicube32 python virtual environment already exists"
fi

# Install kipart into hr2 environment:
echo "Installing kipart into virtual environments"
(workon hr2; if -z `which kipart` ; then pip install $WORKON_HOME/kipart ; fi ; deactivate)
(workon kipart; if -z `which kipart` ; the pip install . ; fi ; deactivate)

# Install kicube32 into hr2 environment:
echo "Installing kipart into virtual environments"
(workon hr2; if -z `which kicube32` ; then pip install . ; fi ; deactivate)
(workon kicube; if -z `which kicube32` ; then pip install $WORONHOME/kicube32 ; fi ; deactivate)
