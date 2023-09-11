# Spot Micro Experiments

* docker-full: a ROS2 dockerfile for use on an RPi4
* spot_ws: a ROS2 workspace containing all custom packages to control a Spot Micro

## Robot Hardware

Originally designed based off of the [Spot Micro NovaSM3](https://novaspotmicro.com/). The electronics have mostly been replaced with just an RPi4.

## Software

Some of this software is based on the [spot-mini-mini repo from OpenQuadruped](https://github.com/OpenQuadruped/spot_mini_mini).

## Usage

* Set up the RPi 4, install docker on it, build the docker image from `docker-full`
* start the docker container and get a shell on it (see instructions in `docker-full` README)
* within docker, run the following

```
cd /home/ros/spot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## TODO:

* test on hardware
* document current nodes
* clean up code
  * make sure nodes/processes finish cleanly on ctl-c
* decide on test features for all spot servos
