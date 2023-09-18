# Spot Micro Experiments

* docker-full: a ROS2 Humble dockerfile for use on an RPi4
* spot_ws: a ROS2 workspace containing all custom packages to control a Spot Micro

## Robot Hardware

Originally designed based off of the [Spot Micro NovaSM3](https://novaspotmicro.com/). The electronics have mostly been replaced with just an RPi4.

You'll want to put together your robot, including servos, before doing anything with this repo. Instead of NovaSM3 electronics, we just use an RPi4 (8GB RAM, 32GB SD) and a PCA9685 breakout board to control the servos.

For commanding the robot, a Logitech F710 gamepad is used.

## Software

Some of this software is based on the [spot-mini-mini repo from OpenQuadruped](https://github.com/OpenQuadruped/spot_mini_mini).

## Usage

NOTE: There are a lot of parameters you may want to tweak for your robot. The most important are probably servo pin numbers, which are hardcoded in `spot_ws/src/joint_ctl/joint_ctl/servo_ctl.py`

* Set up the RPi 4, install docker on it, build the docker image from `docker-full`
* start the docker container and get a shell on it (see instructions in `docker-full` README)
* within docker, run the following

```
cd /home/ros/spot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Check out the launch files in `spot_ws/launch` for examples of what you can do.

## TODO:

* walking gait
* estop into servo controller?
  * rigid-halt or go limp?
* make sure nodes/processes finish cleanly on ctl-c
* electronics diagrams
