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

# For debugging, please turn on the debugging flags below:
# set -x           # Trace execution.
# set -e           # Exit immediately on error result.
# set -o pipefail  # Fail if any commands in a pipeline fail.
# set -u           # Treat unset variables as an error.

# Set up the common install shell variables and define `update_repository` shell function.
source install_start.sh

# Make sure that the master branch is up to date.
update_repository $HR2_HOME

# Inform the user that packages are being installed and 
echo "Installing additional packages."
echo "TO INSTALL UBUNTU PACKAGES, YOU MAY BE PROMPTED FOR YOUR LOGIN PASSWORD..."

# Make sure we have the Ubuntu build-essential package installed (mostly to get make).
if [ -z `which make` ]
then
    echo "**************** Installing Ubuntu build-essential package ..."
    $APT_INSTALL build-essential
else
    echo "Ubuntu build-essential package previously installed"
fi

# Make sure we have Ubuntu git package installed for version control management.
if [ -z `which git` ]
then
    echo "**************** Installing Ubuntu git package for version control management ..."
    $APT_INSTALL git
else
    echo "Ubuntu git package for version control management previously installed."
fi

# Make sure we have Ubunutu vim package for vim the editor installed.
if [ -z `which vim` ]
then
    echo "**************** Installing Ubuntu vim package for the vim editor ..."
    $APT_INSTALL vim
else
    echo "Ubuntu vim package for the vim editor previously installed"
fi

# Make sure we have Ubuntu emacs package for the emacs editor installed.
if [ -z `which emacs` ]
then
    echo "**************** Installing ubuntu emacs package for the emacs editor ..."
    $APT_INSTALL emacs
else
    echo "The Ubuntu emacs package editor of emacs editor previously installed"
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

# Install Ubuntu python3_pip3 package:
if [ -z `which pip3` ]
then
    echo "**************** Installing Ubuntu python3-pip package ..."
    $APT_INSTALL python3-pip
else
    echo "Ubuntu python3-pip package previously installed."
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
if [ -z "$VIRTUALENVWRAPPER_SH" ]
then
    echo "???????????????? VIRTUALENVWRAPPER_SH is empty"
else
    source $VIRTUALENVWRAPPER_SH
fi

if [ -z "`grep virtualenvwrapper.sh $BASHRC`" ]
then
    echo "**************** Add 'source .../virtualenvwrapper.sh' to $BASHRC ..."
    echo "source $VIRTUALENVWRAPPER_SH" >> $BASHRC
else
    echo "source .../virtualenvwrapper.sh already in $BASHRC ."
fi

# Make sure that the $PROJECT_HOME/bin directory is in the $PATH in $BASHRC:
#echo "HR2_HOME=$HR2_HOME"
HR2_HOME_BIN="$HR2_HOME/bin"
#echo "HR2_HOME_BIN=$HR2_HOME_BIN"
HR2_HOME_BIN_IN_BASHRC=`grep $HR2_HOME_BIN $BASHRC`
#echo "HR2_HOME_BIN_IN_BSHRC=$HR2_HOME_BIN_IN_BSHRC"
if [ -z  "$HR2_HOME_BIN_IN_BSHRC" ]
then 
    echo "**************** Adding $HR2_HOME/bin to PATH in $BASHRC ..."
    echo '# Only add $BASHRC to PATH once:' >> $BASHRC
    echo 'case ":$PATH:" in' >> $BASHRC
    echo "  *:$HR2_HOME_BIN:*) ;; # Already in path, do nothing " >> $BASHRC
    echo "  *) export PATH=\$PATH:$HR2_HOME_BIN ;; # Not in path, append it" >> $BASHRC
    echo "esac" >> $BASHRC
    echo "" >> $BASHRC
    echo "!!!!!!!!!!!!!!!! Be sure to do a 'source ~/.bashrc after this to update your PATH.'"
else
    echo "$HR2_HOME/bin was previously added to PATH in $BASHRC." 
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
source $VIRTUALENVWRAPPER_SH

# Create the "hr2" Python virtual environment:
if [ ! -d $WORKON_HOME/hr2 ]
then
    echo "**************** Create hr2 python virtual environment ..."
    $MKVIRTUALENV -a $PROJECT_HOME/hbrc_ros_robot_platform hr2
else
    echo "hr2 Python virtual environment already exists."
fi

### Start configuring git:
# 1. Get the `hub` program for interacting with GitHub.Com.
# 2. Ensure the remote repository named upstream exists.
# 3. Disable pushing any branch to upstream.
# 4. Disable commits to the master branch.
# 5. If there is a GITHUB_ACCOUNT_NAME specified:
#    a. Create the GitHub.com fork
#    b. Set up the staging remote
#    

# Install `hub` program for interfacing with GitHub.Com:
if [ -z `which hub` ]
then
    echo "**************** Installing hub program to interface with GitHub.Com ..."
    sudo snap install hub --classic
else
    echo "hub program was previously installed."
fi

# Make sure that `upstream` remote exists:
if [ `git remote -v | grep -c upstream` == "0" ]
then
    echo "**************** Installing upstream remote for get ..."
    git remote add upstream git@github.com:hbrobotics/hbrc_ros_robot_platform
    #  git remote set-head upstream -a  # This should work, but seems not to for some reason but...
    echo "A"
    #git branch --set-upstream-to=upstream/master master  # Seems to get the job done
    git branch --set-upstream-to=upstream/master master  # Seems to get the job done
else
    echo "Remote named upstream exists for get."
fi

# Make sure that no pushing to upstream can occur:
PRE_PUSH=.git/hooks/pre-push
if [ ! -x $PRE_PUSH ]
then
    echo '**************** Disallow pushes to remote repository named upstream ...' > $PRE_PUSH
    echo '#!/bin/bash' >> $PRE_PUSH
    echo '# Prevent pushes to remote repository named upstream.' >> $PRE_PUSH
    echo '# Incoming shell arguments $1=remote_name $2=remote_url' >> $PRE_PUSH
    echo 'if [ "$1" == "upstream" ] ;' >> $PRE_PUSH
    echo 'then'  >> $PRE_PUSH
    echo '    echo "git push to remote repository named upstream is disallowed."' >> $PRE_PUSH
    echo '    echo "Please do development in another git branch."' >> $PRE_PUSH
    echo '    exit 1' >> $PRE_PUSH
    echo 'fi' >> $PRE_PUSH
    chmod +x $PRE_PUSH
else
    echo "Pushes to remote repository named upstream have been previously disabled."
fi

# Make sure that the master branch disallows commits to the master branch.
PRE_COMMIT=.git/hooks/pre-commit
if [ ! -x $PRE_COMMIT ]
then
    echo "**************** Ensure that commits can not be performed on the master branch ..."
    # This is a more that a little kludgy:
    echo '#!/bin/sh' > $PRE_COMMIT
    echo 'branch="$(git rev-parse --abbrev-ref HEAD)"' >> $PRE_COMMIT
    echo 'if [ "$branch" = "master" ]; then' >> $PRE_COMMIT
    echo '    echo "You can not  commit directly to master branch"' >> $PRE_COMMIT
    echo '    exit 1' >> $PRE_COMMIT
    echo 'fi' >> $PRE_COMMIT
    chmod +x $PRE_COMMIT
else
    echo "Commits can not be performed on the master branch."
fi

# Make sure we have an sshkey:
SSH_KEY_FILE=$HOME/.ssh/id_rsa
if [ -f "$SSH_KEY_FILE" ]
then
    echo "Secure shell encryption key previously generated."
else
    # Since, we don't want any prompting from `ssh-keygen`, see the URL below:
    # https://stackoverflow.com/questions/43235179/how-to-execute-ssh-keygen-without-prompt/45031320
    echo "**************** Creating an secure shell encyption key ..."
    ssh-keygen -q -t rsa -N '' -f $SSH_KEY_FILE 2>/dev/null <<< y >/dev/null
fi

# Create the remote fork:
if [ -n "$GITHUB_ACCOUNT_NAME" ]
then
    # We have a GitHub.Com account name:
    # echo "We have a GitHub.Com account name $GITHUB_ACCOUNT_NAME ."
    if [ -z "`git remote -v | grep staging`" ]
    then
	echo "**************** Creating a remote staging repository fork on GigHub.Com ..."
	echo "You may be prompted password to the GitHub.Com account named $GITHUB_ACCOUNT_NAME..."
	hub fork --remote-name staging --org $GITHUB_ACCOUNT_NAME
	git remote add staging git@github.com:$GITHUB_ACCOUNT_NAME/hbrc_ros_robot_platform.git
    else
	echo "A project fork for staging was previously created on GitHub.Com."
    fi
else
    echo H4
    echo '!!!!!!!!!!!!!!!! No fork project repository until GITHUB_ACCOUT_NAME specfied in ~/.bshrc'
    echo '!!!!!!!!!!!!!!!! Example: export GITHUB_ACCOUNT_NAME=...  # Replace ... with account name'
    echo '!!!!!!!!!!!!!!!! Follow by typing source ~/.bashrc'
fi
