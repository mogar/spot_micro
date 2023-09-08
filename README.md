# Spot Micro Experiments

* docker-full: a ROS2 dockerfile for use on an RPi4
* spot_ws: a ROS2 workspace containing all custom packages to control a Spot Micro

## Robot Hardware

Originally designed based off of the [Spot Micro NovaSM3](https://novaspotmicro.com/). The electronics have mostly been replaced with just an RPi4.

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

* figure out I2C control (ditch busio, basically)
  * use SMBus?? If so, make a wrapper class that can be handed to adafruit PWM ctlr lib?
  ```
  import smbus

  # I2C channel 1 is connected to the GPIO pins
  channel = 1
  address = 0x60 # TODO
  bus = smbus.SMBus(channel)
  ```
  * maybe just replace/rewrite adafruit stuff
    * https://github.com/adafruit/Adafruit_CircuitPython_PCA9685/blob/main/adafruit_pca9685.py
    * https://github.com/adafruit/Adafruit_CircuitPython_ServoKit/blob/main/adafruit_servokit.py
    * https://github.com/adafruit/Adafruit_CircuitPython_Motor/blob/main/adafruit_motor/servo.py#L77
* integrate PWM control into servo_ctl node of joint_ctl package
* update joylistener to make it custom for joy2servodebug