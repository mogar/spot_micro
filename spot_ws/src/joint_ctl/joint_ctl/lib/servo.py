# Based on Adafruit servo libraries
#   See also: 
#   * https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#   * https://github.com/adafruit/Adafruit_CircuitPython_Motor/
# 
#    * https://github.com/adafruit/Adafruit_CircuitPython_PCA9685/blob/main/adafruit_pca9685.py
#    * https://github.com/adafruit/Adafruit_CircuitPython_ServoKit/blob/main/adafruit_servokit.py
#    * https://github.com/adafruit/Adafruit_CircuitPython_Motor/blob/main/adafruit_motor/servo.py#L77


from typing import Optional, Type
from types import TracebackType

import RPi.GPIO as GPIO
from smbus2 import SMBus
# see also: https://www.abelectronics.co.uk/kb/article/1094/i2c-part-4---programming-i-c-with-python

class PcaPwm():
    """
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
    def __init__(self, channel: int, address: int = 0x40):
        self._osc = 25E6 # internal oscillator frequency
        self._reset_pin = 17 # header pin 11, gpio pin 17
        self._address = address
        self._bus = SMBus(channel)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._reset_pin, GPIO.OUT)
        self.enable()

    # TODO: getter and setter
    def frequency(self, freq: int):
        prescale = self._osc/(4096*freq) - 1
        # min of 3   => 1526Hz
        # max of 255 => 24Hz
        clamped_prescale = min(3, max(prescale, 255))
        prescale_bytes = clamped_prescale.to_bytes(1, "little")
        try:
            self._bus.write_byte_data(self._address, 0xFE, prescale_bytes[0])
        except:
            print("ERROR: couldn't write frequency to PWM controller")

    def enable(self):
        GPIO.output(self._reset_pin, GPIO.LOW)

    def disable(self):
        GPIO.output(self._reset_pin, GPIO.HIGH)

    def write_pwm(self, pin: int, pwm: int):
        # NOTE: we always have 0 phase offsets, this means the
        # ON registers are always set to 0. The OFF registers
        # are set based on desired duty cycle. The OFF register
        # should be set via the pwm value is clamped as follows:
        pwm = max(0, min(pwm, 4095))
        # the clamped value determines duty cycle, out of 4095
        # NOTE: there is an individual disable bit in the high register for
        # each pin. It's currently unused.

        pwm_bytes = pwm.to_bytes(2, "little")
        try:
            self._bus.write_byte_data(self._address, (6+4*pin), 0)
            self._bus.write_byte_data(self._address, (6+4*pin + 1), 0)
            self._bus.write_byte_data(self._address, (6+4*pin + 2), pwm_bytes[0]) # off LO
            self._bus.write_byte_data(self._address, (6+4*pin + 3), pwm_bytes[1]) # off HI
        except:
            print("ERROR: couldn't write duty cycle to pwm controller")



class Servo():
    def __init__(self, comms: PcaPwm, servo_id: int, home: int = 1000, min_out: int = 750, max_out: int = 1500) -> None:
        self._comms = comms
        self._servo_id = servo_id
        self._min_out = min_out
        self._max_out = max_out
        self._home = home
        self.set_target(self._home)

    def set_target(self, target: int) -> int:
        # bounds checking
        self._target = min(self._min_out, max(target, self._max_out))
        self._comms.write_pwm(self._servo_id, self._target)
        return self._target

    def get_target(self) -> int:
        return self._target

    # TODO: set max and min values

    # TODO: set_angle, get_angle
    # NOTE: see property and setter here: https://github.com/adafruit/Adafruit_CircuitPython_Motor/blob/main/adafruit_motor/servo.py#L123
