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
    git branch --set-upstream-to=upstream/master master  # Seems to get the job done
else
    echo "Remote named upstream exists for get."
fi

# Make sure that no pushing to upstream is occurs:
if [ `git remote -v | grep -c no-pushing-to-upstream-remote-is-allowed` == "0" ]
then
    echo "**************** Ensuring that nothing can be pushed to upstream remote ..."
    git remote set-url --push upstream no-pushing-to-upstream-remote-is-allowed
    # Next we want to set the `upstream` head to default to `master`:
    echo "here 1"
    git remote set-head upstream master
else
    echo "Nothing can be pushed to upstream remote."
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

set -x
# Create the remote fork:
if [ -n "$GITHUB_ACCOUNT_NAME" ]
then
    echo H1
    if [ -n `git remote -v | grep staging` ]
    then
	echo H2
	echo "**************** Creating a remote staging repository fork on GigHub.Com ..."
	hub fork --remote-name staging --org $GITHUB_ACCOUNT_NAME
	git remote add staging git@github.com:$GITHUB_ACCOUNT_NAME/hbrc_ros_robot_platform.git
        touch $REMOTE_FORK_CREATED
        git remote set-url staging no-pulling-from-staging-remote-is-allowed
    else
	echo H3
	echo "Forked project repository already created on GitHub.Com."
    fi
else
    echo H4
    echo '!!!!!!!!!!!!!!!! No fork project repository until GITHUB_ACCOUT_NAME specfied in ~/.bshrc'
    echo '!!!!!!!!!!!!!!!! Example: export GITHUB_ACCOUNT_NAME=...  # Replace ... with account name'
    echo '!!!!!!!!!!!!!!!! Follow by typing source ~/.bashrc'
fi
