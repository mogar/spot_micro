
from math import radians as rad

import rclpy
from rclpy.node import Node

from spot_interfaces.msg import JointAngles

import motion_control.lib.motion_utils as mu
from motion_control.lib.spot_kinematics import SpotKinematics

import copy

class PoseCycle(Node):
    """
    Class implementing ROS2 Node to cycle through a fixed list of poses.
    """

    def __init__(self) -> None:
        """Setup pubs and subs, initialize null command."""
        super().__init__('PoseCycle')
        self._joint_pub = self.create_publisher(JointAngles, 'target_joints', 10)
        self._current_joints_sub = self.create_subscription(JointAngles, "current_joints", self.current_joints_callback, 10)

        # Set up parameters used in parsing joy messages
        self.declare_parameter('publish_period_s', 0.02)

        # Set up motion parameters
        self.declare_parameter('max_angle_delta_per_s', rad(45.0))
        self.declare_parameter('cycle_time_s', 12)

        # create a timer to publish the command at the right rate
        self._max_angle_delta = self.get_parameter('publish_period_s').value * self.get_parameter('max_angle_delta_per_s').value
        self.create_timer(self.get_parameter('publish_period_s').value, self.joint_publisher)

        self.create_timer(self.get_parameter('cycle_time_s').value, self.cycle_poses)

        # SpotKinematics defaults to correct size and standing pose
        self._kinematics = SpotKinematics()
        # record most recent state command
        self._pose_id = 0
        # record current joint angles
        self._current_joints = JointAngles()

        leg_height = 0.02
        stand = self._kinematics.get_foot_coords()
        fl_up = copy.copy(stand)
        fl_up[0, 1] += leg_height
        fr_up = copy.copy(stand)
        fr_up[1, 1] += leg_height
        bl_up = copy.copy(stand)
        bl_up[2, 1] += leg_height
        br_up = copy.copy(stand)
        br_up[3, 1] += leg_height

        # list of poses to cycle through
        self._pose_list = [
            stand, # 0
            fl_up,
            stand,
            fl_up, # repeat fl_up so we can see when the cycle start
            stand,
            fr_up,
            stand,
            bl_up,
            stand,
            br_up # 9
        ]

        self.get_logger().info("PoseCycle initialized")


    def current_joints_callback(self, msg) -> None:
        """Store the current joint angles for use in calculating target future joint angles."""
        self._current_joints = msg

    def cycle_poses(self):
        """Change to the next pose in the cycle."""
        self._pose_id += 1
        if self._pose_id >= len(self._pose_list):
            self._pose_id = 0
        pose = self._pose_list[self._pose_id]
        self._kinematics.set_foot_coords(pose)
        self.get_logger().info("cycling poses - {}".format(self._pose_id))

    def joint_publisher(self):
        target_angles = self._kinematics.get_joint_angles()
        target_joints = mu.multi_joint_one_step_interp(self._current_joints, mu.np_array_to_joint_angles(target_angles), self._max_angle_delta)
        self._joint_pub.publish(target_joints)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseCycle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
