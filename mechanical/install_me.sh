#!/bin/bash

# Update the repository:
echo "Updating repository ..."
git pull

# Update everything else:
./install_me_main.sh
