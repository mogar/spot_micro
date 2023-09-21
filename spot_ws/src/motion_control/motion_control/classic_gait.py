
import rclpy
from rclpy.node import Node

from spot_interfaces.msg import JointAngles
from geometry_msgs.msg import Twist
from spot_interfaces.msg import StateCmd

from motion_control.lib.state import SitState

class gait_control(Node):
    """
    Class implementing ROS2 Node to convert motion commands into joint angles over time (gait).
    """

    def __init__(self) -> None:
        """Setup pubs and subs, initialize null command."""
        super().__init__('gait_control')
        self._joint_pub = self.create_publisher(JointAngles, 'target_joints', 10)
        self._current_joints_sub = self.create_subscription(JointAngles, "current_joints", self.current_joints_callback, 10)
        self._twist_cmd_sub = self.create_subscription(Twist, "twist", self.cmd_callback, 10)
        self._twist_cmd_sub # prevent unused variable warning
        self._state_cmd_sub = self.create_subscription(StateCmd, "state_cmd", self.state_cmd_callback, 10)
        self._state_cmd_sub # prevent unused variable warning

        # Set up parameters used in parsing joy messages
        self.declare_parameter('publish_period_s', 0.02)

        # Set up gait parameters
        self.declare_parameter('speed_deadzone', 0.01) # TODO: fine tune value
        self.declare_parameter('max_angle_delta_per_s', 30)

        # create a timer to publish the command at the right rate
        self._max_angle_delta = self.get_parameter('publish_period_s').value * self.get_parameter('max_angle_delta_per_s').value
        self.create_timer(self.get_parameter('publish_period_s').value, self.joint_publisher)

        # State for state machine handling motion control
        self._motion_state = SitState()
        # record most recent twist command
        self._cmd = Twist()
        # record most recent state command
        self._state_cmd = StateCmd()
        self._state_cmd.sit = 1 # start sitting
        # record current joint angles
        self._current_joints = JointAngles()

        self.get_logger().info("gait_control initialized")


    def current_joints_callback(self, msg) -> None:
        """Store the current joint angles for use in calculating target future joint angles."""
        self._current_joints = msg

    def cmd_callback(self, msg) -> None:
        """Store the most recent twist command for reference in the timed control loop."""
        self._cmd = msg

    def state_cmd_callback(self, msg) -> None:
        """Store the most recent state command for reference in the timed control loop."""
        self._state_cmd = msg

    def joint_publisher(self):
        self._motion_state = self._motion_state.next_state_from_cmd(self._cmd, self._state_cmd)
        target_joints_msg = self._motion_state.joint_angles_from_cmd(self._current_joints, self._cmd, self._state_cmd, self._max_angle_delta)

        self._joint_pub.publish(target_joints_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = gait_control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
