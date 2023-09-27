""" Forward and Inverse Kinematics for a Spot Micro robot.

This code is very heavily based on the great kinematics library from mike4192. Check it out:
https://github.com/mike4192/spot_micro_kinematics_python/

He wrote his kinematics library based on this paper:
Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017). 
Inverse Kinematic Analysis Of A Quadruped Robot.
International Journal of Scientific & Technology Research. 6.

I didn't read the paper, I just read his code and adapted it for what I wanted.
"""

import numpy as np
import numpy.typing as npt

class LegKinematics():
    def __init__(self, hip_angle_rad: float = 0, elbow_angle_rad: float = 0, wrist_angle_rad: float = 0, hip_len_cm: float = 2, thigh_len_cm: float = 3, shin_len_cm: float = 4, body2leg_transform: npt.NDArray, right_not_left: bool) -> None:
        """Constructor."""
        self.hip_angle_rad = hip_angle_rad
        self.elbow_angle_rad = elbow_angle_rad
        self.wrist_angle_rad = wrist_angle_rad
        
        self.hip_len_cm = hip_len_cm
        self.thigh_len_cm = thigh_len_cm
        self.shin_len_cm = shin_len_cm

        self.body2leg_transform = body2leg_transform
        self.right_not_left = right_not_left
        # TODO: tranformation matrices for joints

    def set_angles(self, hip_angle_rad: float, elbow_angle_rad: float, wrist_angle_rad: float) -> None:
        """Set the angles of the leg and update transformation matrices as needed."""
        self.hip_angle_rad = hip_angle_rad
        self.elbow_angle_rad = elbow_angle_rad
        self.wrist_angle_rad = wrist_angle_rad
        # TODO: update transformation matrices

    def set_transform_to_body(self, body2leg_transform: npt.NDArray) -> None:
        """Set transformation from body pose to leg pose.
        
        This transform is to the top of the leg (the hip joint)."""
        self.body2leg_transform = body2leg_transform

    def get_transform_to_body(self) -> npt.NDArray:
        """Get transformation from body pose to leg pose"""
        return self.body2leg_transform

    def set_foot_pos_in_local_coords(self, x: float, y: float, z: float) -> None:
        """Set position of the foot. Joint angles to achieve the position are calculated via inverse kinematics from the
        input coordinates (leg frame).
        """
        leg_angles = np.zeros(3)
        # TODO: inverse kinematics
        self.set_angles(leg_angles[0], leg_angles[1], leg_angles[2])

    def set_foot_pose_in_body_coords(self, x: float, y: float, z: float) -> None:
        """Set position of the foot. Joint angles to achieve the position are calculated via inverse kinematics from the
        input coordinates (body frame).
        """
        # TODO: convert body coord foot pose to local coord foot pose
        self.set_foot_pos_in_local_coords(0, 0, 0)

    def get_leg_points(self) -> npt.NDArray:
        """Get the coordinates for the four points that define leg pose.

            Point 1: body to hip joint location
            Point 2: hip to thigh joint location
            Point 3: thigh to wrist joint location
            Point 4: end of foot location
        """
        # TODO
        return np.array([0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0])

    def get_foot_pose_in_body_coords(self) -> npt.NDArray:
        """Return the coordinates of the foot's position in the body frame."""
        return np.array([0, 0, 0])

    def get_leg_angles(self) -> npt.NDArray:
        """Return leg angles (hip, elbow, wrist)."""
        return np.array([self.hip_angle_rad,
                         self.elbow_angle_rad,
                         self.wrist_angle_rad])

class SpotKinematics():
    def __init__(self, hip_len_cm: float = 2, thigh_len_cm: float = 3, shin_len_cm: float = 4, body_width_cm: float = 5, body_len_cm: float = 6, body_pitch_rad: float = 0, body_roll_rad: float = 0, body_yaw_rad: float = 0):
        """Initialize dimensions and pose of spot for use in motion calculations.
        """
        self.hip_len_cm = hip_len_cm
        self.thigh_len_cm = thigh_len_cm
        self.shin_len_cm = shin_len_cm
        self.body_width_cm = body_width_cm
        self.body_len_cm = body_len_cm

        # TODO: use SO(3)?
        self.body_pitch_rad = body_pitch_rad
        self.body_roll_rad = body_roll_rad
        self.body_yaw_rad = body_yaw_rad

        # TODO: hold legs as separate objects?
        # TODO: body_transform for each leg
        self.legs = {
            "fl": LegKinematics(hip_angle_rad, elbow_angle_rad, wrist_angle_rad, hip_len_cm, thigh_len_cm, shin_len_cm, body_transform, False),
            "fr": LegKinematics(hip_angle_rad, elbow_angle_rad, wrist_angle_rad, hip_len_cm, thigh_len_cm, shin_len_cm, body_transform, True),
            "bl": LegKinematics(hip_angle_rad, elbow_angle_rad, wrist_angle_rad, hip_len_cm, thigh_len_cm, shin_len_cm, body_transform, False),
            "br": LegKinematics(hip_angle_rad, elbow_angle_rad, wrist_angle_rad, hip_len_cm, thigh_len_cm, shin_len_cm, body_transform, True)
        }

    def get_joint_angles(self) -> npt.NDArray:
        """Gets the current joint angles for the entire robot.

        Joint angles are expected to be an array of:
            (fls, fle, flw;
             frs, fre, frw;
             bls, ble, blw;
             brs, bre, brw)
        """
        return np.vstack((self.legs["fl"].get_leg_angles(),
                          self.legs["fr"].get_leg_angles(),
                          self.legs["bl"].get_leg_angles(),
                          self.legs["br"].get_leg_angles()))

    def set_joint_angles(self, joint_angles: npt.NDArray) -> None:
        """Sets the current joint angles for the entire robot.

        The joints shall be an array of:
            (fls, fle, flw;
             frs, fre, frw;
             bls, ble, blw;
             brs, bre, brw)
        """
        self.legs["fl"].set_leg_angles(joint_angles[0,0], joint_angles[0,1], joint_angles[0,2])
        self.legs["fr"].set_leg_angles(joint_angles[1,0], joint_angles[1,1], joint_angles[1,2])
        self.legs["bl"].set_leg_angles(joint_angles[2,0], joint_angles[2,1], joint_angles[2,2])
        self.legs["br"].set_leg_angles(joint_angles[3,0], joint_angles[3,1], joint_angles[3,2])

    def get_foot_coords(self) -> npt.NDArray:
        """Return the current foot positions relative to body pose.
        """
        return np.vstack((self.legs["fl"].get_foot_pose_in_body_coords(),
                          self.legs["fr"].get_foot_pose_in_body_coords(),
                          self.legs["bl"].get_foot_pose_in_body_coords(),
                          self.legs["br"].get_foot_pose_in_body_coords()))

    def set_foot_coords(self, foot_coords: npt.NDArray) -> None:
        """Sets position of feet relative to body pose.

        Foot position should be a 4x3 array with columns as (x, y, z) positions and rows as the different legs.
        """
        self.legs["fl"].get_foot_pose_in_body_coords(foot_coords[0, 0], foot_coords[0, 1], foot_coords[0, 2])
        self.legs["fr"].get_foot_pose_in_body_coords(foot_coords[1, 0], foot_coords[1, 1], foot_coords[1, 2])
        self.legs["bl"].get_foot_pose_in_body_coords(foot_coords[2, 0], foot_coords[2, 1], foot_coords[2, 2])
        self.legs["br"].get_foot_pose_in_body_coords(foot_coords[3, 0], foot_coords[3, 1], foot_coords[3, 2])

    def get_body_angles(self) -> npt.NDArray:
        """Return the current lean of the body.

        Angles are an array of (roll, pitch, yaw).
        """
        # TODO:
        return np.array([0])

    def set_body_angles(self, body_lean: npt.NDArray) -> None:
        """Sets lean of body (pose) while leaving foot positions fixed.

        Angles are an array of (roll, pitch, yaw).
        """
        # TODO:
        # get current foot position so we can make sure they stay put

        # update the body position

        # update the body to leg transform for each leg

        # set pose of feet in the body frame
        pass

    