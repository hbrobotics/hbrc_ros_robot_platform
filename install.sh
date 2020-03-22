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
PIP_INSTALL="pip install"
WORKON_HOME="$HOME/.virtualenvs"
MKVIRTUALENV="mkvirtualenv -p 3.6" 

# Capture the PROJECT_HOME directory root:
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

# Install Python virtual environments:
if [ ! -n `which virtuaenvwrapper.sh` ]
then
    echo "Installing Python virtual environments ..."
    $PIP_INSTALL virtualenvwrapper
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
if [ ! -d $REPOS/bom_manager ]
then
    echo "Cloning bom_manager ..."
    (cd $REPOS ; git clone https://github.com/waynegramlich/bom_manager.git)
else
    echo "bom_manager previously cloned"
fi

# Clone kicube32:
if [ ! -e $REPOS/kicube32 ]
then
    echo "Cloning kicube32"
    (cd $REPOS ; git clone https://github.com/waynegramlich/bom_manager.git)
else
    echo "kicube32 previously cloned"
fi

# Modify ~/.bashrc to support virtual environments:
if ! grep WORKON_HOME $BASHRC
then
    echo "Add WORKON_HOME to $BASHRC"
    echo "WORKON_HOME=$WORKONHOME" >> $BASHRC
else
    echo "WORKON_HOME is already in $BASHRC"
fi
if ! grep PROJECT_HOME $BASHRC
then
    echo "Add PROJECT_HOME to $BASHRC"
    echo "PROJECT_HOME=$PROJECTHOME" >> $BASHRC
else
    echo "PROJECT_HOME is already in $BASHRC"
fi
if ! grep `which virtualenvwrapper.sh`
then
    echo "Add source .../virtualenvwrapper.sh to $BASHRC"
    echo "source `which virtualenvwrapper.sh`" >> $BASHRC
else
    echo "virutalenvrapper.sh alread in $BASHRC"
fi

# Set up virtual environments:
if ! -d $WORKON_HOME/hr2
then
    echo "Create hr2 python virtual environment"
    $MKVIRTUALENV -a $WORKON_HOME/hr2 hr2
else
    echo "hr2 python virtual environment already exists"
fi
if ! -d $WORKON_HOME/bom_manager
then
    echo "Create hr2 python virtual environment"
    $MKVIRTUALENV -a $WORKON_HOME/bom_manager bom_manager
else
    echo "bom_manager python virtual environment already exists"
fi
if ! -d $WORKON_HOME/kicube32
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
