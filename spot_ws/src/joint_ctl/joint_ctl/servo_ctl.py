
import rclpy
from rclpy.node import Node
from joint_ctl.lib.servo import Servo, PcaPwm

class SpotJoints(Node):

    def __init__(self):
        super().__init__("node_test")

        self._controller = PcaPwm(channel = 20)
        # TODO: max, min, default?
        self._flc_servo = Servo(self._controller, 0) # front-left coxa
        # TODO: other joints

        self.counter_ = 0
        self.get_logger().info("SpotJoints initialized")
        # TODO: callback for messages


def main(args=None):
    print('Hi from joint_ctl.')

    rclpy.init(args=args)
    node = SpotJoints()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
