
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

        # joint rotation limits
        coxa_min = -30.0
        coxa_max = 30.0
        hip_min = -78.0
        hip_max = 78.0
        knee_min = -60.0
        knee_max = 150.0

        # Front Left Leg
        (flc_min, flc_max) = self.get_joint_limits(400, coxa_min, coxa_max)
        self._flc_servo = Servo(self._controller, 3, min_out = flc_min, max_out = flc_max) # front-left coxa
        (flh_min, flh_max) = self.get_joint_limits(400, hip_min, hip_max, True)
        self._flh_servo = Servo(self._controller, 4, min_out = flh_min, max_out = flh_max) # front-left hip
        (flk_min, flk_max) = self.get_joint_limits(360, knee_min, knee_max, True)
        self._flk_servo = Servo(self._controller, 5, min_out = flk_min, max_out = flk_max) # front-left knee
        # Front Right Leg
        (frc_min, frc_max) = self.get_joint_limits(380, coxa_min, coxa_max)
        self._frc_servo = Servo(self._controller, 0, min_out = frc_min, max_out = frc_max) # front-right coxa
        (frh_min, frh_max) = self.get_joint_limits(350, hip_min, hip_max)
        self._frh_servo = Servo(self._controller, 1, min_out = frh_min, max_out = frh_max) # front-right hip
        (frk_min, frk_max) = self.get_joint_limits(400, knee_min, knee_max)
        self._frk_servo = Servo(self._controller, 2, min_out = frk_min, max_out = frk_max) # front-right knee
        # Back Left Leg
        (blc_min, blc_max) = self.get_joint_limits(365, coxa_min, coxa_max)
        self._blc_servo = Servo(self._controller, 9, min_out = blc_min, max_out = blc_max) # back-left coxa
        (blh_min, blh_max) = self.get_joint_limits(420, hip_min, hip_max, True)
        self._blh_servo = Servo(self._controller, 10, min_out = blh_min, max_out = blh_max) # back-left hip
        (blk_min, blk_max) = self.get_joint_limits(370, knee_min, knee_max, True)
        self._blk_servo = Servo(self._controller, 11, min_out = blk_min, max_out = blk_max) # back-left knee
        # Back Right Leg
        (brc_min, brc_max) = self.get_joint_limits(370, coxa_min, coxa_max)
        self._brc_servo = Servo(self._controller, 6, min_out = brc_min, max_out = brc_max) # back-right coxa
        (brh_min, brh_max) = self.get_joint_limits(340, hip_min, hip_max)
        self._brh_servo = Servo(self._controller, 7, min_out = brh_min, max_out = brh_max) # back-right hip
        (brk_min, brk_max) = self.get_joint_limits(390, knee_min, knee_max)
        self._brk_servo = Servo(self._controller, 8, min_out = brk_min, max_out = brk_max) # back-right knee

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

    def get_joint_limits(self, zero_count: int, neg_limit_deg: float, pos_limit_deg: float, invert: bool = False):
        # TODO: get degrees_per_count from a servo object or something?
        degrees_per_count = 0.6
        if not invert:
            return ([rad(neg_limit_deg), int(zero_count + neg_limit_deg/degrees_per_count)],
                    [rad(pos_limit_deg), int(zero_count + pos_limit_deg/degrees_per_count)])
        else:
            return ([rad(neg_limit_deg), int(zero_count - neg_limit_deg/degrees_per_count)],
                    [rad(pos_limit_deg), int(zero_count - pos_limit_deg/degrees_per_count)])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpotJoints()
    rclpy.spin(node)
    # TODO: add method to release RPi.GPIO
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
