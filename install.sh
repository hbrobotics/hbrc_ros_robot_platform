#!/bin/bash

# Shell scripts are notorously difficult to read and debug.
# To aid in reading/debugging this script, there are deliberately plenty of comments.

# For debugging, please turn on the debugging flags below:
set -x           # Trace execution.
set -e           # Exit immediately on error result.
set -o pipefail  # Fail if any commands in a pipeline fail.
set -u           # Treat unset variables as an error.

# Use a macro for invoking `sudo`:
SUDO=sudo -H
INSTALL=$SUDO apt install
PIP_INSTALL=pip install

# Capture the REPOS directory root:
REPOS=`(cd .. ; pwd)`

# Inform the user that packages are being installed and 
echo "Installing additional packages.  You may be prompted for your root password ..."

# Make sure we have the build-essential package installed.
if [ ! -n `which make` ]
    then
        echo "Installing build-essential package ..."
        $INSTALL build-essential
    fi

# Make sure we have openscad installed:
if [ ! -n `which openscad` ]
    then
        echo "Installing openscad package ..."
        $INSTALL openscad
    fi
    
# Make sure we have qcad installed:
if [ ! -n `which qcad` ]
    then
        echo "Installing qcad package ..."
	$INSTALL qcad
    fi

# Install kicad:
if [ ! n `which kicad` ]
    then
        echo "Installing kicad electronic computer aided design package ..."
	echo "Add KiCad to your package repository database:"
        $SUDO add-apt-repository --yes ppa:js-reynaud/kicad-5.1
	echo "Update your package repository database:"
        $SUDO sudo apt update
	echo "Install kicad. This over a gigabyte of code/libraires to download ..."
        $INSTALL --install-recommends kicad
	echo "Install KiCad demo projects:"
        $INSTALL kicad-demo
    fi

# Install python3:
if [ ! -n `which python3` ]
    then
        echo "Installing Python 3 ..."
	$INSTALL python3
    fi

# Install Python virtual environments are installed:
if [ ! -n `which virtuaenvwrapper.sh` ]
    then
        echo "Installing Python virtual environments ..."
	$PIP_INSTALL virtualenvwrapper
    fi

# Modify the 
