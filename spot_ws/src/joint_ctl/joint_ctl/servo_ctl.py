
import rclpy
from rclpy.node import Node
from joint_ctl.lib.servo import Servo, PcaPwm

from spot_interfaces.msg import JointAngles


class SpotJoints(Node):
    """
    Class implementing ROS2 Node to convert to drive Spot servos to desired angles.
    """
    def __init__(self) -> None:
        """Set up servos for all joints. Initiate subscriber for joint angle messages."""
        super().__init__("spot_joints")

        self._controller = PcaPwm(channel = 1)
        # TODO: max, min, default?
        self._fls_servo = Servo(self._controller, 0, min_out = [-90.0, 300], max_out = [90.0, 500]) # front-left shoulder
        # TODO: other joints

        self._joint_sub = self.create_subscription(
            JointAngles,
            'joints',
            self.joint_msg_callback,
            10)
        self._joint_sub  # prevent unused variable warning

        self.get_logger().info("SpotJoints initialized")

    def joint_msg_callback(self, msg) -> None:
        """Send joint angles from received message to servo controller hardware."""
        self.get_logger().info("fls: " + str(msg.fls))
        self._fls_servo.set_angle(msg.fls)
        # TODO: other joints


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpotJoints()
    rclpy.spin(node)
    # TODO: add method to release RPi.GPIO
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
