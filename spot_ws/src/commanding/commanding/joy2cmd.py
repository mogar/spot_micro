
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from spot_interfaces.msg import SpotMotionCommand

class Joy2Cmd(Node):
    """
    Class implementing ROS2 Node to convert joystick commands into user commands for Spot Micro.

    Tested using a Logitech F710. Depends on the spot_interfaces messages package.
    """

    def __init__(self) -> None:
        """Initialize button and stick configuration, setup pubs and subs."""
        super().__init__('joy2cmd')
        self._cmd_pub = self.create_publisher(SpotMotionCommand, 'spot_command', 10)
        self._joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self._joy_sub # prevent unused variable warning

        # Set up parameters used in parsing joy messages
        self.declare_parameter('publish_period_s', 0.02)

        self.declare_parameter('R_stick_side', 3)
        self.declare_parameter('R_stick_fwd', 4)
        self.declare_parameter('R_trigger_bottom', 5)

        self.declare_parameter('scale', 90.0)

        self.declare_parameter('button_switch', 0) # A
        self.declare_parameter('button_estop', 1) # B

        self.get_logger().info("Joy2Cmd initialized")

        # create a timer to publish the command at the right rate
        self.create_timer(self.get_parameter('publish_period_s').value, self.joint_publisher)

        # TODO: conversions from joy to cmd
        self._joy_msg = Joy()

    def joy_callback(self, msg) -> None:
        """Populate and joint servo angles based on an incoming joy message."""
        self._joy_msg = msg
        
    def joint_publisher(self):
        cmd_msg = SpotMotionCommand()

        # TODO: parse joy message into motion command

        self._cmd_pub.publish(cmd_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Joy2Cmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
