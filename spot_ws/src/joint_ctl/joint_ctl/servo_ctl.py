
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
        self._fls_servo = Servo(self._controller, 3, min_out = [-30.0, 450], max_out = [30.0, 350]) # front-left shoulder
        self._fle_servo = Servo(self._controller, 4, min_out = [-78.0, 530], max_out = [78.0, 270]) # front-left elbow
        self._flw_servo = Servo(self._controller, 5, min_out = [-60.0, 460], max_out = [60.0, 260]) # front-left wrist
        # Front Right Leg
        self._frs_servo = Servo(self._controller, 0, min_out = [-30.0, 330], max_out = [30.0, 450]) # front-right shoulder
        self._fre_servo = Servo(self._controller, 1, min_out = [-78.0, 220], max_out = [78.0, 480]) # front-right elbow
        self._frw_servo = Servo(self._controller, 2, min_out = [-60.0, 300], max_out = [60.0, 500]) # front-right wrist
        # Back Left Leg
        self._bls_servo = Servo(self._controller, 9, min_out = [-30.0, 415], max_out = [30.0, 315]) # back-left shoulder
        self._ble_servo = Servo(self._controller, 10, min_out = [-78.0, 550], max_out = [78.0, 290]) # back-left elbow
        self._blw_servo = Servo(self._controller, 11, min_out = [-60.0, 470], max_out = [60.0, 270]) # back-left wrist
        # Back Right Leg
        self._brs_servo = Servo(self._controller, 6, min_out = [-30.0, 320], max_out = [30.0, 420]) # back-right shoulder
        self._bre_servo = Servo(self._controller, 7, min_out = [-78.0, 210], max_out = [78.0, 470]) # back-right elbow
        self._brw_servo = Servo(self._controller, 8, min_out = [-60.0, 290], max_out = [60.0, 490]) # back-right wrist

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
        self._fls_servo.set_angle_deg(msg.fls)
        self._fle_servo.set_angle_deg(msg.fle)
        self._flw_servo.set_angle_deg(msg.flw)
        # front left leg
        self._frs_servo.set_angle_deg(msg.frs)
        self._fre_servo.set_angle_deg(msg.fre)
        self._frw_servo.set_angle_deg(msg.frw)
        # front left leg
        self._bls_servo.set_angle_deg(msg.bls)
        self._ble_servo.set_angle_deg(msg.ble)
        self._blw_servo.set_angle_deg(msg.blw)
        # front left leg
        self._brs_servo.set_angle_deg(msg.brs)
        self._bre_servo.set_angle_deg(msg.bre)
        self._brw_servo.set_angle_deg(msg.brw)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpotJoints()
    rclpy.spin(node)
    # TODO: add method to release RPi.GPIO
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
