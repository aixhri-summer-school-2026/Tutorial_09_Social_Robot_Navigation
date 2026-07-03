# syntax=docker/dockerfile:1

FROM osrf/ros:humble-desktop-full
ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=nav2-socnav
ARG ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

# Install commonly-used tools.
RUN apt-get update && apt-get install --yes \
    curl \
    git \
    iproute2 \
    iputils-ping \
    less \
    mesa-utils \
    net-tools \
    nano \
    rsync \
    software-properties-common \
    sudo \
    tmux \
    tree \
    unzip \
    usbutils \
    vim \
    wget \
    zip \
    zsh


# Install the build tools
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && apt-get update
RUN apt-get update && apt-get install -y \
    build-essential \
    jq \
    python3-pip 

# Install the dependecies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && apt install -y \
    ros-$ROS_DISTRO-navigation2\
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-libg2o \
    ros-$ROS_DISTRO-joint-state-publisher \
    libopenblas-dev \
    libeigen3-dev\
    libqt5gui5 \
    libsqlite3-dev \
    libqt5widgets5 \
    libzmq3-dev \
    libqt5core5a \
    qtbase5-dev \
    libsdl2-dev \
    libsdl-image1.2-dev \
    libfreeimage-dev \
    gosu \
    xterm \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/${USER_NAME}/socnav_sim/src/socnav_sim
COPY .ci/socnav_sim /home/${USER_NAME}/socnav_sim/src/socnav_sim
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /home/${USER_NAME}/socnav_sim && \
    colcon build --symlink-install

COPY .ci/.bashrc /home/${USER_NAME}/.bashrc

COPY .ci/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]