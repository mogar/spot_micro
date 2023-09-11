
import rclpy
from rclpy.node import Node
from joint_ctl.lib.servo import Servo, PcaPwm

from spot_interfaces.msg import JointAngles


class SpotJoints(Node):

    def __init__(self):
        super().__init__("spot_joints")

        self._controller = PcaPwm(channel = 20)
        # TODO: max, min, default?
        self._fls_servo = Servo(self._controller, 0) # front-left shoulder
        # TODO: other joints

        self._joint_sub = self.create_subscription(
            JointAngles,
            'joints',
            self.joint_msg_callback,
            10)
        self._joint_sub  # prevent unused variable warning

        self.get_logger().info("SpotJoints initialized")

    def joint_msg_callback(self, msg):
        self._fls_servo.set_target(msg.fls)
        # TODO: other joints


def main(args=None):
    print('Hi from joint_ctl.')

    rclpy.init(args=args)
    node = SpotJoints()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
