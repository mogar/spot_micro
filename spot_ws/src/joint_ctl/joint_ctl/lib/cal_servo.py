
from servo import Servo, PcaPwm


controller = PcaPwm(channel = 1)

# intentionally wide limits, these probably won't actually correspond to 
# the real angles here, but we're using targets instead of angles for control
# anyway
servo = Servo(controller, 0, min_out = [-90.0, 0], max_out = [90.0, 1500])
servo.set_target(500) # "safe" starting point

while True:
    target = input("target: ")
    servo.set_target(int(target))