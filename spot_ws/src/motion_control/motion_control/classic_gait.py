
import rclpy
from rclpy.node import Node

from spot_interfaces.msg import JointAngles
from geometry_msgs.msg import Twist


class gait_control(Node):
    """
    Class implementing ROS2 Node to convert motion commands into joint angles over time (gait).
    """

    def __init__(self) -> None:
        """Setup pubs and subs, initialize null command."""
        super().__init__('gait_control')
        self._joint_pub = self.create_publisher(JointAngles, 'joints', 10)
        self._spot_cmd_sub = self.create_subscription(Twist, "twist", self.command_callback, 10)
        self._spot_cmd_sub # prevent unused variable warning

        # Set up parameters used in parsing joy messages
        self.declare_parameter('publish_period_s', 0.01) # TODO: fine tune this value

        # Set up gait parameters
        self.declare_parameter('speed_deadzone', 0.1) # TODO: fine tune valu

        self.get_logger().info("gait_control initialized")

        # create a timer to publish the command at the right rate
        self.create_timer(self.get_parameter('publish_period_s').value, self.joint_publisher)

        # record most recent command
        self._cmd = Twist()

    def command_callback(self, msg) -> None:
        """Store the most recent command for reference in the timed control loop."""
        self._cmd = msg

    def joint_publisher(self):
        joint_msg = JointAngles()

        # TODO: gait command

        self._joint_pub.publish(joint_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = gait_control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
