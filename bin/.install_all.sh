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

# Refresh the repositiory:
echo "**************** Refreshing the hbrc_ros_robot_platform repository ..."
git pull

# Now install the electrical stuff:
echo "**************** Installing the electrical portion of the project ..."
(cd electrical ; ./install_ee.sh )

# Now install the mechancal stuff:
echo "**************** Installing the mechanical porition of the project ..."
(cd mechanical ;  ./install_me.sh )

# Remind people to update their virtual environments:
echo "****************"
echo "Please type 'source ~/.bashrc' to enable python virtual environments."
echo "Next type 'workon hr2' to enter the 'hr2' virtual environment."
echo "To exit the hr2 python virtual environment, type 'deactivate'."
