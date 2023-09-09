
from joint_ctl.lib.servo import Servo, PcaPwm

def main():
    controller = PcaPwm(channel = 20)
    # TODO: max, min, default?
    flc_servo = Servo(controller, 0) # front-left coxa

    print('Hi from joint_ctl.')


if __name__ == '__main__':
    main()
