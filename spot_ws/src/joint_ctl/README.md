# Joint Control via I2C-commanded Servos

This package is intended to control a set of servos via I2C. The servos should be wired up to a PCA9685 (you can use a breakout board like [this one from Adafruit](https://www.adafruit.com/product/815)).

## Dependencies

You can get these via pip:

* RPi.GPIO

## Raspberry Pi I2C setup

On the Pi, you may need to modify `/boot/config.txt` to uncomment the line for `dtparam=i2c_arm=on` and reboot.

## Hardware Connection and Verification

Connect the Controller to your Raspberry Pi with this pinout:

* Pi 3V3 (Pi4 pin1) to driver VCC
* Pi GND (Pi4 pin6) to driver GND
* Pi SCL (Pi4 pin5) to driver SCL
* Pi SDA (Pi4 pin3) to driver SDA
* Pi GPIO17 (Pi4 pin11) to driver enable

The library expects the PWM controller to have I2C address 0x40

To verify your servo driver is connected correctly and usable, run:

```
i2cdetect -y 1
```

You should see it appear in the address table printed out.