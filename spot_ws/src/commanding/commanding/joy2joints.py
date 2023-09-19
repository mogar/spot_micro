
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
        self._joint_pub = self.create_publisher(JointAngles, 'target_joints', 10)
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

        self.get_logger().info("Joy2Joints initialized")

        # create a timer to publish the command at the right rate
        self.create_timer(self.get_parameter('publish_period_s').value, self.joint_publisher)

        # leg 0: FL
        # leg 1: FR
        # leg 2: BL
        # leg 3: BR
        self._leg_id = 0
        self._num_legs = 4
        self._switch_pressed = False

        # starting default angles
        self._shoulder_angle = 0.0
        self._elbow_angle = 0.0
        self._wrist_angle = 0.0

    def joy_callback(self, msg) -> None:
        """Populate and joint servo angles based on an incoming joy message."""

        # It's useful to uncomment the below and run this node standalone if you're having
        # issues with button/axis identification on your gamepad. Just run it and view the
        # output while pressing the different buttons, and sticks.
        # self.get_logger().info("axes: " + str(msg.axes) + ", buttons: " + str(msg.buttons))

        # switch leg via button press
        if msg.buttons[self.get_parameter('button_switch').value]:
            if not self._switch_pressed:
                self._switch_pressed = True
                self._leg_id += 1
                self.get_logger().info("leg id: " + str(self._leg_id))
                if self._leg_id >= self._num_legs:
                    self._leg_id = 0
        else:
            self._switch_pressed = False

        # get control for all servos in active leg
        shoulder = msg.axes[self.get_parameter('R_stick_fwd').value]
        elbow =    msg.axes[self.get_parameter('R_stick_side').value]
        wrist =    msg.axes[self.get_parameter('R_trigger_bottom').value]
        # convert to degrees
        self._shoulder_angle = shoulder*self.get_parameter('scale').value
        self._elbow_angle = elbow*self.get_parameter('scale').value
        self._wrist_angle = wrist*self.get_parameter('scale').value

    def joint_publisher(self):
        joint_msg = JointAngles()

        if self._leg_id == 0: # FL
            joint_msg.fls = self._shoulder_angle
            joint_msg.fle = self._elbow_angle
            joint_msg.flw = self._wrist_angle
        elif self._leg_id == 1: # FR
            joint_msg.frs = self._shoulder_angle
            joint_msg.fre = self._elbow_angle
            joint_msg.frw = self._wrist_angle
        elif self._leg_id == 2: # BL
            joint_msg.bls = self._shoulder_angle
            joint_msg.ble = self._elbow_angle
            joint_msg.blw = self._wrist_angle
        else: #self._leg_id == 3 - BR
            joint_msg.brs = self._shoulder_angle
            joint_msg.bre = self._elbow_angle
            joint_msg.brw = self._wrist_angle

        self._joint_pub.publish(joint_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Joy2Joints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
