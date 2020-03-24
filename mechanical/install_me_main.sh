#!/bin/bash

# Shell scripts are notorously difficult to read and debug.
# To aid in reading/debugging this script, there are deliberately plenty of comments.

# Make sure all of the common stuff has already been done:
../install_common.sh

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
    echo "Installing openscad package ..."
    $APT_ADD_REPOSITORY ppa:openscad/releases
    $APT_UPDATE
    $APT_INSTALL openscad
else
    echo "openscad previously installed."
fi
    
# Make sure markdown to HTML convert is installed:
if [ -z "`which markdown`" ]
then
    echo "Installing markdown to HTML converter ..."
    $APT_INSTALL markdown
else
    echo "markdown to HTML converter previously installed."
fi

# Install qcad:
if [ -z "`which qcad`" ]
then
    echo "Installing qcad package ..."
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
    echo "Installing Python flake8 code style checker ..."
    (workon hr2; $PIP_INSTALL flake8 ; deactivate)
else
    echo "Python flake8 code style checker previously installed."
fi

# Install Python pydocstyle documentation style checker:
if [ -z "`which pydocstyle`" ]
then
    echo "Installing Python pydocstyle documentation style checker ..."
    (workon hr2; $PIP_INSTALL pydocstyle ; deactivate)
else
    echo "Python pydocstyle documentation style checker previously installed."
fi

# Remind people to update their virtual environments:
echo "Please type 'source ~/.bashrc' to enable python virtual environments."
echo "Next type 'workon hr2' to enter the 'hr2' virtual environment."
echo "To exit the hr2 python virtual environment, type 'deactivate'."


