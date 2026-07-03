#!/usr/bin/env bash

set -e

USER_NAME=${USER_NAME:-nav2-socnav}

if ! id "$USER_NAME" &>/dev/null; then
    groupadd -f -g "$HOST_GID" "$USER_NAME"
    useradd -u "$HOST_UID" -g "$HOST_GID" -s /bin/bash "$USER_NAME"
    adduser $USER_NAME sudo && echo '%sudo ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers 

fi

chown -R "$HOST_UID:$HOST_GID" "/home/$USER_NAME"

source /opt/ros/humble/setup.bash
source /home/nav2-socnav/socnav_sim/install/setup.bash

exec gosu "$USER_NAME" "$@"
