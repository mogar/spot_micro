
from math import radians as rad

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
        
        # TODO: parametrize servo pin IDs?

        # Front Left Leg
        self._flc_servo = Servo(self._controller, 3, min_out = [rad(-30.0), 450], max_out = [rad(30.0), 350]) # front-left coxa
        self._flh_servo = Servo(self._controller, 4, min_out = [rad(-78.0), 530], max_out = [rad(78.0), 270]) # front-left hip
        self._flk_servo = Servo(self._controller, 5, min_out = [rad(-60.0), 460], max_out = [rad(90.0), 110]) # front-left knee
        # Front Right Leg
        self._frc_servo = Servo(self._controller, 0, min_out = [rad(-30.0), 330], max_out = [rad(30.0), 450]) # front-right coxa
        self._frh_servo = Servo(self._controller, 1, min_out = [rad(-78.0), 220], max_out = [rad(78.0), 480]) # front-right hip
        self._frk_servo = Servo(self._controller, 2, min_out = [rad(-60.0), 300], max_out = [rad(90.0), 650]) # front-right knee
        # Back Left Leg
        self._blc_servo = Servo(self._controller, 9, min_out = [rad(-30.0), 415], max_out = [rad(30.0), 315]) # back-left coxa
        self._blh_servo = Servo(self._controller, 10, min_out = [rad(-78.0), 550], max_out = [rad(78.0), 290]) # back-left hip
        self._blk_servo = Servo(self._controller, 11, min_out = [rad(-60.0), 470], max_out = [rad(90.0), 120]) # back-left knee
        # Back Right Leg
        self._brc_servo = Servo(self._controller, 6, min_out = [rad(-30.0), 320], max_out = [rad(30.0), 420]) # back-right coxa
        self._brh_servo = Servo(self._controller, 7, min_out = [rad(-78.0), 210], max_out = [rad(78.0), 470]) # back-right hip
        self._brk_servo = Servo(self._controller, 8, min_out = [rad(-60.0), 290], max_out = [rad(90.0), 640]) # back-right knee

        self._target_joint_sub = self.create_subscription(
            JointAngles,
            'target_joints',
            self.joint_msg_callback,
            10)
        self._target_joint_sub  # prevent unused variable warning
        self._current_joint_pub = self.create_publisher(JointAngles, 'current_joints', 10)


        self.get_logger().info("SpotJoints initialized")

    def joint_msg_callback(self, msg) -> None:
        """Send joint angles from received message to servo controller hardware."""

        # We track actual angles and send them out for use in motion planning, etc.
        # This is not actually a great way to do this, because it's open loop and time-ignorant.
        # We just get the actual commanded angle from the servo, so if it takes a long time
        # (relative to motion planning) for the servo to move there this could cause inaccuracies.
        # We also are trusting that the motor is actually able to move the joint to that angle, which
        # it may not be able to depending on the environment.
        current_joint_angles = JointAngles()

        # front left leg
        current_joint_angles.flc = self._flc_servo.set_angle_rad(msg.flc)
        current_joint_angles.flh = self._flh_servo.set_angle_rad(msg.flh)
        current_joint_angles.flk = self._flk_servo.set_angle_rad(msg.flk)
        # front right leg
        current_joint_angles.frc = self._frc_servo.set_angle_rad(msg.frc)
        current_joint_angles.frh = self._frh_servo.set_angle_rad(msg.frh)
        current_joint_angles.frk = self._frk_servo.set_angle_rad(msg.frk)
        # back left leg
        current_joint_angles.blc = self._blc_servo.set_angle_rad(msg.blc)
        current_joint_angles.blh = self._blh_servo.set_angle_rad(msg.blh)
        current_joint_angles.blk = self._blk_servo.set_angle_rad(msg.blk)
        # back right leg
        current_joint_angles.brc = self._brc_servo.set_angle_rad(msg.brc)
        current_joint_angles.brh = self._brh_servo.set_angle_rad(msg.brh)
        current_joint_angles.brk = self._brk_servo.set_angle_rad(msg.brk)

        self._current_joint_pub.publish(current_joint_angles)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpotJoints()
    rclpy.spin(node)
    # TODO: add method to release RPi.GPIO
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
