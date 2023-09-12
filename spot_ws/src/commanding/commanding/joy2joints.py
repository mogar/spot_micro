
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

        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_linear_z', 2)

        self.declare_parameter('axis_angular', 5)

        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)

        self.declare_parameter('button_switch', 0)
        self.declare_parameter('button_estop', 1)

        self.get_logger().info("Joy2Joints initialized")

    def joy_callback(self, msg) -> None:
        """Populate and send a joint servo message based on an incoming joy message."""
        joint_msg = JointAngles()

        # TODO: control of any servo (switch via button press)
        x_linear = msg.axes[self.get_parameter('axis_linear_x').get_parameter_value().integer_value]
        if x_linear > 0:
            joint_msg.fls = 90.0
        else:
            joint_msg.fls = 0.0

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
