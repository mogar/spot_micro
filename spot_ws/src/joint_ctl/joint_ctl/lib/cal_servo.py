
import time
from servo import Servo, PcaPwm


controller = PcaPwm(channel = 1)

# intentionally wide limits, these probably won't actually correspond to 
# the real angles here, but we're using targets instead of angles for control
# anyway
servo = Servo(controller, 2)
time.sleep(0.01)
servo.set_target(400) # "safe" starting point

while True:
    target = input("target: ")
    servo.set_target(int(target))