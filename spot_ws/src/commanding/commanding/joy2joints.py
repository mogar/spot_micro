
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from spot_interfaces.msg import JointAngles

class Joy2Joints(Node):
    """
    Class implementing ROS2 Node to convert joystick commands directly into joint angles.

    Tested using a Logitech F710. Depends on the spot_interfaces messages package.
    """

    def __init__(self) -> None:
        """Initialize button and stick configuration, setup pubs and subs."""
        super().__init__('joy2joints')
        self._joint_pub = self.create_publisher(JointAngles, 'joints', 10)
        self._joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self._joy_sub # prevent unused variable warning

        # Set up parameters used in parsing joy messages
        self.declare_parameter('frequency', 200.0)

        self.declare_parameter('R_stick_side', 3)
        self.declare_parameter('R_stick_fwd', 4)
        self.declare_parameter('R_trigger_bottom', 5)

        self.declare_parameter('scale', 90.0)

        self.declare_parameter('button_switch', 0) # A
        self.declare_parameter('button_estop', 1) # B

        self.get_logger().info("Joy2Joints initialized")

    def joy_callback(self, msg) -> None:
        """Populate and send a joint servo message based on an incoming joy message."""

        # It's useful to uncomment the below and run this node standalone if you're having
        # issues with button/axis identification on your gamepad. Just run it and view the
        # output while pressing the different buttons, and sticks.
        # self.get_logger().info("axes: " + str(msg.axes) + ", buttons: " + str(msg.buttons))

        joint_msg = JointAngles()

        # TODO: control of any servo (switch via button press)
        r_side = msg.axes[self.get_parameter('R_stick_side').value]
        # convert to degrees
        ankle_angle = r_side*self.get_parameter('scale').value

        #self.get_logger().info("input: " + str(r_side) + ", angle: " + str(ankle_angle))
        joint_msg.fls = ankle_angle

        self._joint_pub.publish(joint_msg)


def main(args=None) -> None:
    print('Hi from joy2joints.')

    rclpy.init(args=args)
    node = Joy2Joints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
