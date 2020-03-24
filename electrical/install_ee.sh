#!/bin/bash

# Update the repository:
echo "Updating repository ..."
git pull

# Update everything else:
source install_ee_main.sh
