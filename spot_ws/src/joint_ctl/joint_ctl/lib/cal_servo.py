
import time
from spot_joints import spot_joint_servos
from math import radians as rad
from math import pi

time.sleep(0.01)
for _, servo in spot_joint_servos.items():
    servo.set_angle_rad(0.0) # "safe" starting point

print("Specify input as a string: joint,t,angle.")
print("Joint should be the name of an individual joint (like \"flc\").")
print("t should be a type (either \'r\' for radians, \'d\' for degrees, or \'c\' for counts)")
print("angle should be a value corresponding to the type chosen")

while True:
    target = input("target: ")
    s_name, t, angle = target.split(',')

    if t == 'r':
        spot_joint_servos[s_name].set_angle_rad(float(angle))
    elif t == 'd':
        spot_joint_servos[s_name].set_angle_rad(float(angle)*pi/180.0)
    elif t == 'c':
        spot_joint_servos[s_name].set_target(int(angle))
    else:
        print("invalid input: {}".format(target))
        break
