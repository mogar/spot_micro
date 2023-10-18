
import time
from servo import Servo, PcaPwm
from math import radians as rad


controller = PcaPwm(channel = 1)

flc_servo = Servo(controller, 3, min_out = [rad(-30.0), 350], max_out = [rad(30.0), 450]) # front-left coxa
flh_servo = Servo(controller, 4, min_out = [rad(-78.0), 530], max_out = [rad(78.0), 270]) # front-left hip
flk_servo = Servo(controller, 5, min_out = [rad(-60.0), 460], max_out = [rad(90.0), 110]) # front-left knee

time.sleep(0.01)
flc_servo.set_angle_rad(0.0) # "safe" starting point
flh_servo.set_angle_rad(0.0)
flk_servo.set_angle_rad(0.0)

while True:
    target = input("target: ")
    flc_servo.set_angle_rad(float(target))
    flh_servo.set_angle_rad(float(target))
    flk_servo.set_angle_rad(float(target))