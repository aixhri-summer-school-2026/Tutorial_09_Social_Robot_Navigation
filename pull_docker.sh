#!/bin/bash
set -e  # Stop the script on error

# Pull the image
docker pull aixhrisummerschool2026/aixhri-summer-school-2026:Tutorial_09_Social_Robot_Navigation

# Rename the image
docker tag aixhrisummerschool2026/aixhri-summer-school-2026:Tutorial_09_Social_Robot_Navigation ros-humble-socnav

# Delete the untagged image
docker rmi aixhrisummerschool2026/aixhri-summer-school-2026:Tutorial_09_Social_Robot_Navigation