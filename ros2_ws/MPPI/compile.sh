#!/bin/bash

type=${1:-Release}

# Build the selected packages (for development)
if command -v colcon &> /dev/null
then
    colcon build
else
    echo "colcon not found!"
    exit 1
fi
