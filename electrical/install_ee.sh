#!/bin/bash

# Shell scripts are notorously difficult to read and debug.
# To aid in reading/debugging this script, there are deliberately plenty of comments.

# Make sure all of the common stuff has already been done:
source ../install_common.sh

# For debugging, please turn on the debugging flags below:
set -x           # Trace execution.
set -e           # Exit immediately on error result.
set -o pipefail  # Fail if any commands in a pipeline fail.
set -u           # Treat unset variables as an error.

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
    echo "Cloning bom_manager repository into $PROJECT_HOME ."
    (cd $PROJECT_HOME ; git clone https://github.com/waynegramlich/bom_manager.git)
else
    echo "bom_manager previously cloned into $PROJECT_HOME ."
fi

# Clone kicube32:
if [ ! -e $PROJECT_HOME/kicube32 ]
then
    echo "Cloning kicube32 repository into $PROJECT_HOME ."
    (cd $PROJECT_HOME ; git clone https://github.com/waynegramlich/kicube32.git)
else
    echo "kicube32 previously cloned into $PROJECT_HOME ." 
fi

# The virtual wrapper script breaks under debugging, so turn everything off:
set +x           # Trace execution.
set +e           # Exit immediately on error result.
set +o pipefail  # Fail if any commands in a pipeline fail.
set +u           # Treat unset variables as an error.

# Install kipart into hr2 environment:
echo "Installing kipart into hr2 virtual environment."
(workon hr2;
 if -z `which kipart` ;
 then
     pip install $WORKON_HOME/kipart ;
 fi ;
 deactivate)

# Install kicube32 into hr2 environment:
echo "Installing kicube32 into hr2 virtual environment."
(workon hr2;
 if -z `which kicube32` ;
 then pip install $WORKON_HOME/kicube32 ;
 fi ;
 deactivate)

