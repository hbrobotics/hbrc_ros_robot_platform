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
    echo "openscad previously installed."
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

