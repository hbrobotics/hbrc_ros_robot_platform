#!/bin/bash

# This file is intended to be run from other scripts via a `source install_start.sh` command.
# It sets up all of the environment variables and makes darn sure that the
# hbrc_ros_robot_platform repository is up-to-date.  This piece of code can be safely
# executed multiple times.
#
# This script defines some commonly used shell variables *AND* makes sure that the master
# branch is up to date.  See below about update the master branch.

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
PROJECT_HOME=`pwd | sed s,/hbrc_ros_robot_platform.*,,g`
HR2_HOME=$PROJECT_HOME/hbrc_ros_robot_platform

# The master branch is configured to be "read-only in the sense that it is *NEVER* modified on
# the local machine.  Modifications on other branches are allowed, but not to the local master.
# What this means is that it is always safe to do a `git pull` into the master branch.  We do
# over think this, we just attempt to checkout master and do the pull.  If it fails, we let
# you know that the checkout failed:

# Step 1: Determine if git is installed:
WHICH_GIT=`which git`
echo "WHICH_GIT=$WHICH_GIT"
if [ -z "$WHICH_GIT" ]
then
    echo "git program is not installed yet."
else
    # Step 2: Determine if we are in a repository:
    REPOSITORY_ROOT=`git rev-parse --show-toplevel 2> /dev/null`
    echo "REPOSITORY_HOME=$REPOSITORY_ROOT"
    if [ -z "$REPOSITORY_ROOT" ]
    then
	echo "Current working directory is not in a git repository."
    else
        # Step 3: Deterimine if the `upstream` remote has been set up yet.  If not we
	# are done until the `upstream` remote is setup by the `install_common.sh` file:
        UPSTREAM_REMOTE=`git remote -v | sed "s,\t.*\$,,g" | sort | uniq | grep upstream`
	echo "UPSTREAM_REMOTE=$UPSTREAM_REMOTE"

	if [ -z "$UPSTREAM_REMOTE" ]
	then
	    echo "The remote git repository called upstream has not been set up yet."
	else
	    # Step 4: Determine if we have a `master` branch.  If not, things
	    # are pretty screwed up, but there is no point in proceeding:
            MASTER_BRANCH=`git branch -v | sed "s,^..,,g" | sed "s, .*,,g" | grep master`
            echo "MASTER_BRANCH=$MASTER_BRANCH"
        
            if [ -z "$MASTER_BRANCH"  ]
	    then
		echo "There is no master branch in this repository.  Weird."
	    else
		# Step 5: Get current branch name and a list of uncommitted files.
		# If we are not on the `master` branch we will temporarily switch over `master`,
		# update master and return the current branch.  This is easily said, but
		# not so easily done:
		CURRENT_BRANCH=`git rev-parse --abrev-ref HEAD`
		echo "CURRENT_BRANCH=$CURRENT_BRANCH"
		UNCOMMITTED_FILES=`git diff --name-only HEAD`
		echo "UNCOMMITTED_FILES=$UNCOMMITTED_FILES"

                # Step 6: Now figure out if we need to change branches:
		if [ "$CURRENT_BRANCH" != "master" ]
		then
		    if [ -n "$UNCOMMITTED_FILES" ]
		    then
			# Step 6: We have uncommitted files in this repository. We need to
			# temporarily "stash" them before changing over to `master` branch:
			echo "Temporarily saving uncommitted files in this branch ($CURRENT_BRANCH)"
			git stash
		    fi
		    
		    # Step 7: Temporarily switch over to `master` branch:
		    echo "Temporarily switching over to master branch"
		    git checkout master

		    # Step 8: Finally perform the git pull from the upstream repository into
		    # the master branch.  Because the local master branch has been configured
		    # to disallow commits, in theory, there can be no can be no merge conflicts:
		    git pull upstream master
		    echo "master branch should be up to date"

		    # Step 9: Now we return to the original branch we started (if necessary):
		    if [ "$CURRENT_BRANCH" != "master" ]
		    then
			echo "Switch from master branch back to $CURRENT_BRANCH branch."
			git checkout $CURRENT_BRANCH

			# Step 10: Now restore the branch to the way it was:
			git stash pop

			# Now figure out if a merge from master will have any conflicts.
			# This is done by doing a `git merge --no-commit`, followed by a
			# `git merge -abort`.  The return code from the `--no-commit`
			# is used to decide whether to do the actual merge or not.
			# If no conlicts will occur, perform the merge to this branch:
			if git merge --no-comit master master
			then
			    echo "this branch"
			    git merge --abort
			    echo "Merging any updates from the master branch to this branch."
			    echo "git merge master"
			else
			    echo "that branch"
			    git merge --abort
			    echo "There would be merge conflits if we merge with the master branch."
			    echo "Please add your modified files into your branch with git add."
			    echo "When they are all added, please do a git commit."
			    echo "Then rerun the shell script again."
			fi
		    fi
		fi
	    fi
	fi
    fi
fi


