
from math import radians as rad

import rclpy
from rclpy.node import Node
from spot_interfaces.msg import JointAngles

from joint_ctl.lib.spot_joints import spot_joint_servos

class SpotJoints(Node):
    """
    Class implementing ROS2 Node to convert to drive Spot servos to desired angles.
    """
    def __init__(self) -> None:
        """Set up servos for all joints. Initiate subscriber for joint angle messages."""
        super().__init__("spot_joints")

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
        current_joint_angles.flc = spot_joint_servos["flc"].set_angle_rad(msg.flc)
        current_joint_angles.flh = spot_joint_servos["flh"].set_angle_rad(msg.flh)
        current_joint_angles.flk = spot_joint_servos["flk"].set_angle_rad(msg.flk)
        # front right leg
        current_joint_angles.frc = spot_joint_servos["frc"].set_angle_rad(msg.frc)
        current_joint_angles.frh = spot_joint_servos["frh"].set_angle_rad(msg.frh)
        current_joint_angles.frk = spot_joint_servos["frk"].set_angle_rad(msg.frk)
        # back left leg
        current_joint_angles.blc = spot_joint_servos["blc"].set_angle_rad(msg.blc)
        current_joint_angles.blh = spot_joint_servos["blh"].set_angle_rad(msg.blh)
        current_joint_angles.blk = spot_joint_servos["blk"].set_angle_rad(msg.blk)
        # back right leg
        current_joint_angles.brc = spot_joint_servos["brc"].set_angle_rad(msg.brc)
        current_joint_angles.brh = spot_joint_servos["brh"].set_angle_rad(msg.brh)
        current_joint_angles.brk = spot_joint_servos["brk"].set_angle_rad(msg.brk)

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
