#!/usr/bin/env bash

set -o errexit
set -o nounset
set -ex

IMAGE="ros-humble-socnav"

# source ./env.sh

# docker buildx build --rm=true --progress=plain --build-arg="HOME=/home/$CONTAINER_NAME" --build-arg="UID=1000" --build-arg="GID=1000" --build-arg="USER_NAME=$CONTAINER_NAME" -t "$IMAGE" .
docker build --rm=true --progress=plain -t "$IMAGE" -f .ci/Dockerfile .
docker system prune -f
