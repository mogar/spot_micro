# Joint Control via I2C-commanded Servos

This package is intended to control a set of servos via I2C. The servos should be wired up to a PCA9685 (you can use a breakout board like [this one from Adafruit](https://www.adafruit.com/product/815)).

## Dependencies

You can get these via pip:

* RPi.GPIO
* adafruit-circuitpython-pca9685
* adafruit-circuitpython-servokit

## Hardware Connection and Verification

Connect the Controller to your Raspberry Pi with this pinout:

* Pi 3V3 (Pi4 pin1) to driver VCC
* Pi GND (Pi4 pin6) to driver GND
* Pi SCL (Pi4 pin5) to driver SCL
* Pi SDA (Pi4 pin3) to driver SDA

The library expects the PWM controller to have I2C address 0x

To verify your servo driver is connected correctly and usable, run:

```
i2cdetect -y 1
```

You should see it appear in the address table printed out.