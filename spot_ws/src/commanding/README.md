# Commanding

User input for commanding the spot robot.

## Joystick

This is tested with a Logitech F710. To verify that your gamepad is working before running any nodes in here, do the following:

```
sudo apt-get install jstest-gtk
jstest --event /dev/input/js0
```

This should give you a running list of your joystick events, allowing you to verify that it works and that the buttons and sticks are numbered how you expect.