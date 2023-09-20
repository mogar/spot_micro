
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from spot_interfaces.msg import StateCmd

class Joy2Cmd(Node):
    """
    Class implementing ROS2 Node to convert joystick commands into user commands for Spot Micro.

    Tested using a Logitech F710. Depends on the spot_interfaces messages package.
    """

    def __init__(self) -> None:
        """Initialize button and stick configuration, setup pubs and subs."""
        super().__init__('joy2cmd')
        self._estop_pub = self.create_publisher(Bool, 'estop', 10)
        self._cmd_pub = self.create_publisher(Twist, 'twist', 10)
        self._state_cmd_pub = self.create_publisher(StateCmd, 'state_cmd', 10)
        self._joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self._joy_sub # prevent unused variable warning

        # store joy message for later conversion into command messages
        self._joy_msg = None

        # estop tracking
        self._estop = False
        self._estop_button_pressed = False
        # Start in sitting position
        self._sit = 1
        self._sit_button_pressed = False

        # Set up parameters used in parsing joy messages
        self.declare_parameter('publish_period_s', 0.02)

        self.declare_parameter('R_stick_side', 3)
        self.declare_parameter('R_stick_fwd', 4)
        self.declare_parameter('R_trigger_bottom', 5)

        self.declare_parameter('lin_x_scale', 1.0)
        self.declare_parameter('lin_y_scale', 1.0)
        self.declare_parameter('lin_z_scale', 1.0)

        self.declare_parameter('L_stick_side', 6)
        self.declare_parameter('L_stick_fwd', 7)
        self.declare_parameter('L_trigger_bottom', 2)
        self.declare_parameter('pluspad_fwd', 1)

        self.declare_parameter('ang_x_scale', -1.0)
        self.declare_parameter('ang_y_scale', -1.0)
        self.declare_parameter('ang_z_scale', -1.0)

        self.declare_parameter('button_switch', 0) # A
        self.declare_parameter('button_estop', 1) # B

        # create a timer to publish the command at the right rate
        self.create_timer(self.get_parameter('publish_period_s').value, self.cmd_publisher)

        self.get_logger().info("Joy2Cmd initialized")

    def joy_callback(self, msg) -> None:
        """Store incoming joy message for later use."""
        self._joy_msg = msg

    def cmd_publisher(self):
        if self._joy_msg is None:
            # wait until we have a real message to publish any twist
            return

        # estop message
        # TODO: timed debouncing here if needed
        # TODO: function for debouncing and press/hold?
        if self._joy_msg.buttons[self.get_parameter('button_estop').value]:
            if not self._estop_button_pressed:
                self._estop_button_pressed = True
                self._estop = not self._estop
        else:
            self._estop_button_pressed = False

        estop_msg = Bool()
        estop_msg.data = self._estop
        self._estop_pub.publish(estop_msg)

        # State Commands
        if self._joy_msg.buttons[self.get_parameter('button_switch').value]:
            if not self._sit_button_pressed:
                self._sit_button_pressed = True
                if self._sit == 0:
                    self._sit = 1
                else:
                    self._sit = 0
        else:
            self._sit_button_pressed = False

        state_cmd = StateCmd()
        state_cmd.sit = self._sit
        self._state_cmd_pub.publish(state_cmd)

        # Twist message
        cmd_msg = Twist()

        y = self._joy_msg.axes[self.get_parameter('R_stick_fwd').value]
        x = self._joy_msg.axes[self.get_parameter('R_stick_side').value]
        z = self._joy_msg.axes[self.get_parameter('R_trigger_bottom').value]

        x *= self.get_parameter('lin_x_scale').value
        y *= self.get_parameter('lin_y_scale').value
        z *= self.get_parameter('lin_z_scale').value

        cmd_msg.linear.x = x
        cmd_msg.linear.y = y
        cmd_msg.linear.z = z

        ang_y = self._joy_msg.axes[self.get_parameter('L_stick_fwd').value]
        ang_x = self._joy_msg.axes[self.get_parameter('L_stick_side').value]
        ang_z = self._joy_msg.axes[self.get_parameter('pluspad_fwd').value]

        ang_x *= self.get_parameter('ang_x_scale').value
        ang_y *= self.get_parameter('ang_y_scale').value
        ang_z *= self.get_parameter('ang_z_scale').value

        cmd_msg.angular.x = ang_x
        cmd_msg.angular.y = ang_y
        cmd_msg.angular.z = ang_z

        self._cmd_pub.publish(cmd_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Joy2Cmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
