#! /usr/bin/env bash

set -o errexit
set -o nounset

# source env.sh

IMAGE="ros-humble-socnav"

xhost +local:docker > /dev/null

args=(
    --rm
    --interactive
    --tty
    --privileged 
    --network=host
    --env TERM=xterm-256color
    --hostname="$HOSTNAME"

    --ipc=host
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro"
    --volume="/dev/dri:/dev/dri:ro"
    --env DISPLAY="$DISPLAY"
    --env HOST_UID=$(id -u)
    --env HOST_GID=$(id -g)

    --volume="$HOME:$HOME"
    --mount type=bind,src="$PWD/ros2_ws",dst="/home/nav2-socnav/socnav_ws"
    --workdir="/home/nav2-socnav"

    --runtime=nvidia
    --gpus all
    --env NVIDIA_DRIVER_CAPABILITIES="all"

    "$IMAGE"
    "$SHELL"
    )

if ! docker container inspect -f '{{.State.Running}}' "nav2-socnav" &> /dev/null; then
    docker run "${args[@]}"		
fi

