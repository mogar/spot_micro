"""
Lightweight servo control library for use with the PCA9685.

For a good PCA9685 breakout board, check out [AdaFruit](https://www.adafruit.com/product/815)

You can find the datasheet for the PCA9685 [here](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)
"""

from typing import Optional, Type
from types import TracebackType

import time
import RPi.GPIO as GPIO
from smbus2 import SMBus

from rclpy import logging

class PcaPwm():
    """
    Handle communication and high level control of a PCA9685 PWM controller.

    Assumes use of the primary I2C interface and one GPIO pin to control a PCA9685
    * Pi 3V3 (Pi4 pin1) to driver VCC
    * Pi GND (Pi4 pin6) to driver GND
    * Pi SCL (Pi4 pin5) to driver SCL
    * Pi SDA (Pi4 pin3) to driver SDA
    * Pi GPIO17 (Pi4 pin11) to driver enable

    0x00: mode reg 1 - bit 7 is restart
    0x01: mode reg 2
    All PWM outputs have 4 bytes for power control
      register addresses are (6+4*id) + byte_num
      so id = 0 would have addresses 0x06 to 0x09
      the addresses are (low to high) on_lo, on_hi, off_lo, off_hi
    0xFE: frequency prescalar
    """
    def __init__(self, channel: int, address: int = 0x40) -> None:
        self._osc = 25E6 # internal oscillator frequency
        self._mode_addx = 0x00
        self._mode_restart = 0x80
        self._mode_sleep = 0x10
        self._prescale_addx = 0xFE

        self._reset_pin = 17 # header pin 11, gpio pin 17

        self._address = address
        self._bus = SMBus(channel)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._reset_pin, GPIO.OUT)

        self.set_frequency(60) # based on NovaSM3 experiments 
        self.enable()

    def get_frequency(self):
        return self._osc/(4095*(self._clamped_prescale + 1))

    def set_frequency(self, freq: int) -> None:
        prescale = self._osc/(4096*freq) - 1
        # min of 3   => 1526Hz
        # max of 255 => 24Hz
        self._clamped_prescale = int(max(3, min(prescale, 255)))
        try:
            # read mode1
            mode1 = self._bus.read_byte_data(self._address, self._mode_addx)
            # write to set it to not restart and sleep
            temp_mode = (mode1 & ~self._mode_restart) | self._mode_sleep
            self._bus.write_byte_data(self._address, self._mode_addx, temp_mode)

            # set prescale
            self._bus.write_byte_data(self._address, self._prescale_addx, self._clamped_prescale)
            
            # disable sleep
            new_mode = mode1 & ~self._mode_sleep
            self._bus.write_byte_data(self._address, self._mode_addx, new_mode)

            # delay 5ms and restart
            time.sleep(0.005)
            new_mode = new_mode | self._mode_restart
            self._bus.write_byte_data(self._address, self._mode_addx, new_mode)
        except:
            logging.get_logger("PcaPwm").info("ERROR: couldn't write frequency to PWM controller")

    def enable(self) -> None:
        """Enable PWM output by driving the gpio low."""
        GPIO.output(self._reset_pin, GPIO.LOW)

    def disable(self) -> None:
        """Disable PWM output by driving the gpio high."""
        GPIO.output(self._reset_pin, GPIO.HIGH)

    def write_pwm(self, pin: int, pwm: int) -> None:
        """Set the specified pin on the PWM controller to turn on at the given pwm value.

        See the datasheet of the PCA9685 for detailed coverage of PWM calculations.

        This module assumes we always have 0 phase offsets, this means the ON registers
        are always set to 0. The OFF registers are set based on desired duty cycle. The
        controller will count from 0 to 4095 at the specified frequency (see above).
        When it hits 0, the pin will turn on. When it hits the value passed in via the
        `pwm` argument, then the pin will turn off. This is how the duty cycle is set.
        """
        # clamp the PWM input to valid values
        pwm = int(max(0, min(pwm, 4095)))
        # the clamped value determines duty cycle, out of 4095
        # NOTE: there is an individual disable bit in the high register for
        # each pin. It's currently unused.

        pwm_lo = pwm & 0xFF
        pwm_hi = (pwm >> 8) & 0xFF
        try:
            self._bus.write_byte_data(self._address, (6+4*pin), 0)
            self._bus.write_byte_data(self._address, (6+4*pin + 1), 0)
            self._bus.write_byte_data(self._address, (6+4*pin + 2), pwm_lo) # off LO
            self._bus.write_byte_data(self._address, (6+4*pin + 3), pwm_hi) # off HI
        except:
            logging.get_logger("PcaPwm").info("ERROR: couldn't write duty cycle to pwm controller")



class Servo():
    """Manage state for a specific PWM pin connected to a servo."""
    def __init__(self, comms: PcaPwm, servo_id: int, min_out: list[int] = [90, 750], max_out: list[int] = [90, 1500], home: float = 0.0) -> None:
        """Initialize parameters of the servo and drive it to home."""
        self._comms = comms
        self._servo_id = servo_id
        
        # limits are tuples of [angle, pwm value]
        # we use them for linear interpolation when determining new pwm values from angle inputs
        self._min_out = min_out
        self._max_out = max_out
        self._home = home
        self.set_angle(self._home)

    def set_target(self, target: int) -> int:
        """Set the desired pwm value of the servo (not angle).

        Return the actual value set (after accounting for clamping)."""
        # bounds checking
        self._target = max(self._min_out[1], min(target, self._max_out[1]))
        self._comms.write_pwm(self._servo_id, self._target)
        return self._target

    def get_target(self) -> int:
        """Return the current pwm value (not angle) of the servo."""
        return self._target

    def angle_to_target(self, angle: float) -> int:
        # linear interp between min/max
        target = (self._min_out[1] - self._max_out[1])/(self._min_out[0] - self._max_out[0]) * \
                (angle - self._max_out[0]) + self._max_out[1]
        target = int(target)
        return target
    
    def target_to_angle(self, target: int) -> float:
        # linear interp between min/max
        angle = (self._min_out[0] - self._max_out[0])/(self._min_out[1] - self._max_out[1]) * \
                (target - self._max_out[1]) + self._max_out[0]
        return angle

    def set_angle(self, angle: float) -> float:
        angle = max(self._min_out[0], min(angle, self._max_out[0]))
        self.set_target(self.angle_to_target(angle))
        return angle

    def get_angle(self) -> float:
        return self.target_to_angle(self._target)

    # NOTE: see property and setter here: https://github.com/adafruit/Adafruit_CircuitPython_Motor/blob/main/adafruit_motor/servo.py#L123
