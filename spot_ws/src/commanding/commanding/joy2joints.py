
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

        # leg 0: FL
        # leg 1: FR
        # leg 2: BL
        # leg 3: BR
        self._leg_id = 0
        self._switch_pressed = 0

    def joy_callback(self, msg) -> None:
        """Populate and send a joint servo message based on an incoming joy message."""

        # It's useful to uncomment the below and run this node standalone if you're having
        # issues with button/axis identification on your gamepad. Just run it and view the
        # output while pressing the different buttons, and sticks.
        # self.get_logger().info("axes: " + str(msg.axes) + ", buttons: " + str(msg.buttons))

        joint_msg = JointAngles()

        # TODO: switch leg via button press (debounced)

        # get control for all servos in active leg
        shoulder = msg.axes[self.get_parameter('R_stick_fwd').value]
        elbow =    msg.axes[self.get_parameter('R_stick_side').value]
        wrist =    msg.axes[self.get_parameter('R_trigger_bottom').value]
        # convert to degrees
        shoulder_angle = shoulder*self.get_parameter('scale').value
        elbow_angle = elbow*self.get_parameter('scale').value
        wrist_angle = wrist*self.get_parameter('scale').value

        if self._leg_id == 0: # FL
            joint_msg.fls = shoulder_angle
            joint_msg.fls = elbow_angle
            joint_msg.fls = wrist_angle
        elif self._leg_id == 1: # FR
            joint_msg.frs = shoulder_angle
            joint_msg.frs = elbow_angle
            joint_msg.frs = wrist_angle
        if self._leg_id == 2: # BL
            joint_msg.bls = shoulder_angle
            joint_msg.bls = elbow_angle
            joint_msg.bls = wrist_angle
        else: #self._leg_id == 3 - BR
            joint_msg.brs = shoulder_angle
            joint_msg.brs = elbow_angle
            joint_msg.brs = wrist_angle

        self._joint_pub.publish(joint_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Joy2Joints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
