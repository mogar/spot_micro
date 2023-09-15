
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
        
        # Front Left Leg
        self._fls_servo = Servo(self._controller, 3, min_out = [-90.0, 402], max_out = [90.0, 282]) # front-left shoulder
        self._fle_servo = Servo(self._controller, 4, min_out = [-90.0, 537], max_out = [90.0, 207]) # front-left elbow
        self._flw_servo = Servo(self._controller, 5, min_out = [-90.0, 370], max_out = [90.0, 128]) # front-left wrist
        # Front Right Leg
        self._frs_servo = Servo(self._controller, 0, min_out = [-90.0, 314], max_out = [90.0, 434]) # front-right shoulder
        self._fre_servo = Servo(self._controller, 1, min_out = [-90.0, 185], max_out = [90.0, 515]) # front-right elbow
        self._frw_servo = Servo(self._controller, 2, min_out = [-90.0, 365], max_out = [90.0, 607]) # front-right wrist
        # Back Left Leg
        self._bls_servo = Servo(self._controller, 9, min_out = [-90.0, 402], max_out = [90.0, 282]) # back-left shoulder
        self._ble_servo = Servo(self._controller, 10, min_out = [-90.0, 446], max_out = [90.0, 116]) # back-left elbow
        self._blw_servo = Servo(self._controller, 11, min_out = [-90.0, 358], max_out = [90.0, 116]) # back-left wrist
        # Back Right Leg
        self._brs_servo = Servo(self._controller, 6, min_out = [-90.0, 329], max_out = [90.0, 449]) # back-right shoulder
        self._bre_servo = Servo(self._controller, 7, min_out = [-90.0, 236], max_out = [90.0, 566]) # back-right elbow
        self._brw_servo = Servo(self._controller, 8, min_out = [-90.0, 278], max_out = [90.0, 520]) # back-right wrist

        self._joint_sub = self.create_subscription(
            JointAngles,
            'joints',
            self.joint_msg_callback,
            10)
        self._joint_sub  # prevent unused variable warning

        self.get_logger().info("SpotJoints initialized")

    def joint_msg_callback(self, msg) -> None:
        """Send joint angles from received message to servo controller hardware."""
        # front left leg
        self._fls_servo.set_angle(msg.fls)
        self._fle_servo.set_angle(msg.fle)
        self._flw_servo.set_angle(msg.flw)
        # front left leg
        self._frs_servo.set_angle(msg.frs)
        self._fre_servo.set_angle(msg.fre)
        self._frw_servo.set_angle(msg.frw)
        # front left leg
        self._bls_servo.set_angle(msg.bls)
        self._ble_servo.set_angle(msg.ble)
        self._blw_servo.set_angle(msg.blw)
        # front left leg
        self._brs_servo.set_angle(msg.brs)
        self._bre_servo.set_angle(msg.bre)
        self._brw_servo.set_angle(msg.brw)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpotJoints()
    rclpy.spin(node)
    # TODO: add method to release RPi.GPIO
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
