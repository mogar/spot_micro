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

### Testing

Run unit tests and view results with something like

```
colcon test --packages-select motion_control
colcon test-result --all --verbose
```

You can run tests on only a single package (change the name to whatever you want). You can also run tests on all packages.

## Reference Frames

For the robot as a whole:
* +X is forward
* +Y is up-down
* +Z is right (CHECK THIS?)

## Terminology notes

Most mammals that we interact with have ball joints for hips. The Spot Micro instead has two hinges that collectively serve the same purpose.
These two hinges are pretty close to each other, but the small linear distance between them does introduce another transform into the robot kinematics.
In order to distinguish between joints, we call the joint closest to the body the "coxa" and the joint second from the body the "hip".

| Joint Term | Description |
| ---------- | ----------- |
| coxa       | First joint between body and a leg. |
| hip        | Second joint in leg, between coxa and knee. |
| knee       | Final joint in leg. Backwards from how a human knee would bend. |
| foot       | The end of the lowest rigid link in the leg. |

Leg links similarly have specific naming:

| Leg link | Description |
| pelvis   | The short link between coxa and hip joints. |
| thigh    | The upper leg link, between hip and knee. |
| shin     | The lower leg link, between knee and ground. |

In order to identify specific joints in the robot, we sometimes used TLAs. For example, the front-left coxa would be called the "flc" and so forth.

## TODO:

* multi-axis motion smoothing is wonky
* get walking working with direction (backwards, angled, turn)
* document frames of reference
  * leg frame
* estop into servo controller?
  * rigid-halt or go limp?
* make sure nodes/processes finish cleanly on ctl-c
* electronics diagrams
