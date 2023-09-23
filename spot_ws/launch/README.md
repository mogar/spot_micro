# Launch Files

These files launch a set of ROS nodes together to accomplish various tasks.

To use any of these, make sure you've sourced the workspace setup file and then run something like this:

```
spot_ws$ ros2 launch launch/<launch_file_name>.py
```

## Calibration and hardware testing

### servo_testing_launch.py

Run this to set up the F710 gamepad to directly control servos. This isn't great for moving the robot around in space, but it is very helpful for calibrating servos and verifying wiring.

* TODO: calibration procedure

## Simple Walking

### simple_walker_launch.py

Basic remote control via gamepad of a walking spot. No sensors or autonomy, just direct remote control.
