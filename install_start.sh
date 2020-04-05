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

# This file is not expected to be executed as a stand-alone shell script.  Instead
# it is meant to be included directly via a `source install_start.sh` command.
#
# This file defines sets up some environment variables and defines a `master_update` function.
# The repository update function, ensures that the master branch of the current git repository
# is up-to-date *AND* it merges any of the up-dates into the current branch, if they can
# be merged with out conflicts.
#
# A note about pronouns.  We and us refer to the developers and maintainers of this software
# and you refers to the user of the software.

# This script defines some commonly used shell variables *AND* makes sure that the master
# branch is up to date.  See below about update the master branch.

# Define some commonly used macros:
SUDO="sudo -H"
APT_ADD_REPOSITORY="sudo add-apt-repository --yes"
APT_INSTALL="$SUDO apt install --yes"
APT_UPDATE="$SUDO apt update"
if [ -z "$HOME" ]
then
    echo '!!!!!!!!!!!!!!!! The HOME environment variable is not set.  This is really bad.'
fi
MKVIRTUALENV="mkvirtualenv -p 3.6" 
BASHRC="$HOME/.bashrc"
PIP_INSTALL="pip3 install"
WORKON_HOME="$HOME/.virtualenvs"

# Capture the PROJECTS_HOME directory root:
PROJECT_HOME=`pwd | sed s,/hbrc_ros_robot_platform.*,,g`
HR2_HOME=$PROJECT_HOME/hbrc_ros_robot_platform

# The master branch is configured to be "read-only in the sense that it is *NEVER* modified on
# the local machine.  Modifications on other branches are allowed, but not to the local master.
# What this means is that it is always safe to do a `git pull` into the master branch.  We do
# over think this, we just attempt to checkout master and do the pull.  If it fails, we let
# you know that the checkout failed:

# Usage repository_update( repository_directory )
# Returns 0 for success and > 0 for errors
function update_repository() {
    # Step 1: Extract the REPOSITORY_DIRECTORY function argument:
    REPOSITORY_DIRECTORY=$1
    #debug: echo "REPOSITORY_DIRECTORY=$REPOSITORY_DIRECTORY"
    ERROR_MESSAGE='Update of repostitory for "$REPOSITORY_DIRECTORY" will not happen.'
    CWD=`pwd` # CWD = Current Working Directory

    # Step 2: Ensure that we have `git` is installed:
    WHICH_GIT=`which git`
    #debug: echo "WHICH_GIT=$WHICH_GIT"
    if [ -z "$WHICH_GIT" ]
    then
	echo '!!!!!!!!!!!!!!!! git program is not installed yet.'
        echo "$ERROR_MESSAGE"
	return 1
    fi

    # Step 3: Ensure that REPOSITORY_DIRECTORY exists and is a directory.
    if [ ! -d "$REPOSITORY_DIRECTORY" ]
    then
	echo '!!!!!!!!!!!!!!!! "$REPOSITORY_DIRECTORY" is not a directory.'
        echo "ERROR_MESSAGE"
	return 2
    fi

    # Step 4: Change over to REPOSITORY_DIRECTORY and find the REPOSITORY_ROOT.
    # From here on, before 
    cd "$REPOSITORY_DIRECTORY"
    # The `2>/dev/null` suppresses error messages
    REPOSITORY_ROOT=`git rev-parse --show-toplevel 2>/dev/null`
    #debug: echo "REPOSITORY_ROOT=$REPOSITORY_ROOT"
    if [ -z "$REPOSITORY_ROOT" ]
    then
	echo '!!!!!!!!!!!!!!!! "$REPOSITORY_DIRECTORY" is not part of a git repository.'
        echo "$ERROR_MESSAGE"
        cd "$CWD"
	return 3
    fi

    # Step 3: Deterimine if the `upstream` remote has been set up yet.  If not,  we are done
    # until the `upstream` remote is setup. `git remote` outputs  a list of remotes and we
    # can search for `upstream`:
    UPSTREAM_REMOTE=`git remote | grep upstream`
    #debug: echo "UPSTREAM_REMOTE=$UPSTREAM_REMOTE"
    if [ -z "$UPSTREAM_REMOTE" ]
    then
	echo '!!!!!!!!!!!!!!!! There is no remote repository named "upstream"' \
	     'defined for repository "$REPOSITORY_ROOT"'
	echo "$ERROR_MESSAGE"
	cd "$CWD"
	return 4
    fi
    
    # Step 4: Determine if we have a `master` branch.  If not, things pretty screwed up.
    # `git branch -v` returns an alphabetized list of branches of the form:
    #
    #       branch1
    #     * branch2
    #       ...
    #       branchN
    #
    # The currently active branch will have an asterisk in front of it.
    # We feed the the output of `git branch` through `sed` to strip off the first two
    # characters and seach for `master`:
    MASTER_BRANCH=`git branch | sed "s,^..,,g" | grep master`
    #debug: echo "MASTER_BRANCH=$MASTER_BRANCH"
    if [ -z "$MASTER_BRANCH" ]
    then
	echo '!!!!!!!!!!!!!!!! There is no master branch in repository "$REPOSITORY_ROOT".'
	echo "$ERROR_MESSAGE"
	cd "$CWD"
	return 5
    fi

    # Step 5: Get current branch name and a list of uncommitted files.  If we are not on the
    #`master` branch we will temporarily switch over to the `master` branch, attempt to update
    # master and return the current branch.  This is easily said, not not so easily done.
    # If we are already on the `master` branch we can avoid the hassle of switching branches:
    # For reference:
    #   https://stackoverflow.com/questions/6245570/how-to-get-the-current-branch-name-in-git
    # explained how to get the CURRENT_BRANCH name.  It is obscure:
    CURRENT_BRANCH=`git rev-parse --abbrev-ref HEAD`
    #debug: echo "CURRENT_BRANCH=$CURRENT_BRANCH"
    # Likewise getting the UNCOMMITTED_FILES is pretty obscrue as well:
    UNCOMMITTED_FILES=`git diff --name-only HEAD`
    #debug: echo "UNCOMMITTED_FILES=$UNCOMMITTED_FILES"
    echo "Starting to update the master branch."
    if [ "$CURRENT_BRANCH" != "master" ]
    then
	# Step 6: We have uncommitted files in this repository. We need to
	# temporarily "stash" them before changing over to `master` branch:
	if [ -n "$UNCOMMITTED_FILES" ]
	then
	    echo "Temporarily saving uncommitted files for $CURRENT_BRANCH branch."
	    git stash > /dev/null # Suppress the messages.
	fi

	# Step 7: Temporarily switch over to `master` branch:
	echo "Temporarily switching over to master branch from $CURRENT_BRANCH branch."
	git checkout master > /dev/null 2>/dev/null # Suppress the messages
    fi

    # Step 8: Finally perform the git pull from the upstream repository into
    # the master branch.  Because the local master branch has been configured
    # to disallow commits, in theory, there can be no can be no merge conflicts:
    git pull upstream master 2>/dev/null 1>/dev/null # Suppress the output
    echo "master branch should be up to date"
	
    # Step 9: Now we return to the original branch we started (if necessary):
    if [ "$CURRENT_BRANCH" != "master" ]
    then
	echo "Switch from master branch back to $CURRENT_BRANCH branch."
	git checkout $CURRENT_BRANCH > /dev/null 2>/dev/null # Suppess the messages

        # Step 10: Now restore the branch to the way it was:
	echo "Restore any previously uncommitted files."
	git stash pop > /dev/null  # `> /dev/null` suppresses an unneeded `git status`

        # We are back in CURRENT_BRANCH with any uncommitted files restored.
	# The following is a references (* is the one selected):
	#
	#     * https://code-maven.com/git-check-for-conflicts-before-merge
	#       https://stackoverflow.com/questions/501407/is-there-a-git-merge-dry-run-option
	#       https://stackoverflow.com/questions/964876/head-and-orig-head-in-git
	#
        # Now figure out if a merge from master will have any conflicts.
	# This is done by doing a `git merge --no-commit`, followed by a
	# `git merge -abort`.  The return code from the `--no-commit`
	# is used to decide whether to do the actual merge or not.
	# If no conlicts will occur, perform the merge to this branch:
	echo "Performing test merge from master branch to $CURRENT_BRANCH."
	if git merge --no-commit --no-ff master > /dev/null 2>/dev/null # Suppress messages
	then
	    echo "Test merge shows no problems.  Undo test merge and do it for real."
	    git merge --abort 2>/dev/null  # `2>/dev/null` suppresses "fatal: ...." message
	    echo "Performing the actual merge from the master branch to $CURRENT_BRANCH branch."
	    git merge master --commit --no-ff --no-edit > /dev/null 2>/dev/null # Suppress messages
	else
	    echo "Test merge shows there will be issue merging master branch" \
		 " to $CURRENT_BRANCH branch."
	    echo "Undoing the test merge."
	    git merge --abort
	    echo "Please add your modified files into your branch with git add."
	    echo "When they are all added, please do a git commit."
	    echo "Then rerun the shell script again."
	fi
    fi

    # We are done, so return to CWD and return:
    cd "$CWD"
    return 0
}

update_repository "."

