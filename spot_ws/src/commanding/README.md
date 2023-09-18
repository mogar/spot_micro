# Commanding

User input for commanding the spot robot.

## Joystick

This is tested with a Logitech F710. To verify that your gamepad is working before running any nodes in here, do the following:

```
sudo apt-get install jstest-gtk
jstest --event /dev/input/js0
```

This should give you a running list of your joystick events, allowing you to verify that it works and that the buttons and sticks are numbered how you expect.

If you're running this package in a docker container, make sure that `/dev/input/event0` and `/dev/input/js0` are shared with the container.

You can also check what joysticks/gamepads are available to the `joy` ROS2 node with:

```
ros2 run joy joy_enumerate_devices
```

If that doesn't list any devices, you may need to add permissions on your input device via:

```
sudo chmod a+rw /dev/input/*
```
