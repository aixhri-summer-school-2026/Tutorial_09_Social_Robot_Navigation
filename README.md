# Socially Aware Robot Navigation (AIxHRI Summer School 2026)

Socially aware robot navigation in Nav2 a custom MPPI planner and CoHAN.

## Prerequisites

- Docker and Docker Compose installed (see [docker setup guide](https://github.com/aixhri-summer-school-2026/docker-nvidia-tuto)).
- Linux host with permission to run `sudo`.

## Setup
Clone the repo and pull the submodules.

```
git clone https://github.com/aixhri-summer-school-2026/Tutorial_09_Social_Robot_Navigation.git
cd Tutorial_09_Social_Robot_Navigation
git submodule update --init --recursive
```

## Pull and Run the docker containers

Pull the docker

```
./pull_docker.sh
```

Run the docker in a shell

```
./run-docker.sh
```

## Build CoHAN
In the docker shell, run the following

```
cd socnav_ws/CoHAN-Nav2
./compile.sh       
```

## Build MPPI
In the docker shell, run the following

```
cd socnav_ws/MPPI
./compile.sh
```

Run visualization for MPPI
```
cd Tutorial_09_Social_Robot_Navigation/docker
docker compose up
```
