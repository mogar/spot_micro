"""Forward and Inverse Kinematics for a Spot Micro robot.

This code is very heavily based on the great kinematics library from mike4192. Check it out:
https://github.com/mike4192/spot_micro_kinematics_python/

One important note is that the default link lengths I use are different from his. I also
modified variables names to be more intelligible, and some kinematics to match how I've
calibrated my own spot.

He wrote his kinematics library based on this paper:
Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017). 
Inverse Kinematic Analysis Of A Quadruped Robot.
International Journal of Scientific & Technology Research. 6.
"""

from math import sqrt, cos, sin, atan2, pi

import numpy as np
import numpy.typing as npt

from rclpy import logging


class LegKinematics():
    """Perform forward and inverse kinematics for a single leg of Spot Micro.

    All lengths are in meters and all angles are in radians.

        base: joint 0 - the fixed point at which the leg hip rotates on the body
        coxa: joint 1 - the point at which the upper leg rotates around the hip, with orientation relative to hip
        hip: joint 2 - the point at which the upper leg rotates around the hip, but in cooardinates relative to upper leg
        knee: joint 3 - the point at which the lower leg rotates with respect to the upper leg
        foot: joint 4 - the tip of the foot
    """

    def __init__(self, body2leg_transform: npt.NDArray, pelvis_len_m: float, thigh_len_m: float, shin_len_m: float, leg_angles_rad: npt.NDArray, name: str = "leg") -> None:
        """Constructor."""
        # TODO: clamps on angles
        self.coxa_angle_rad = leg_angles_rad[0]
        self.hip_angle_rad = leg_angles_rad[1]
        self.knee_angle_rad = leg_angles_rad[2]

        self.pelvis_len_m = pelvis_len_m
        self.thigh_len_m = thigh_len_m
        self.shin_len_m = shin_len_m

        self.body2base_transform = body2leg_transform

        # update tranformation matrices for joints
        self.update_body2coxa_transform()
        self.update_coxa2hip_transform()
        self.update_hip2knee_transform()
        self.update_knee2foot_transform()

        self._leg_name = name

    def update_body2coxa_transform(self) -> None:
        """Update the transform from the body to the coxa. Uses pre-set angles and lengths."""
        self.body2coxa_transform = \
            np.array([[cos(self.coxa_angle_rad), -sin(self.coxa_angle_rad), 0.0, -self.pelvis_len_m*cos(self.coxa_angle_rad)],
                      [sin(self.coxa_angle_rad),  cos(self.coxa_angle_rad), 0.0, -self.pelvis_len_m*sin(self.coxa_angle_rad)],
                      [                     0.0,                       0.0, 1.0,                                         0.0],
                      [                     0.0,                       0.0, 0.0,                                         1.0]])

    def update_coxa2hip_transform(self) -> None:
        """Update the transform from the coxa to the hip. This transform is actually just a constant rotation."""
        self.coxa2hip_transform = \
            np.array([[ 0.0,  0.0, -1.0,  0.0],
                      [-1.0,  0.0,  0.0,  0.0],
                      [ 0.0,  1.0,  0.0,  0.0],
                      [ 0.0,  0.0,  0.0,  1.0]])

    def update_hip2knee_transform(self) -> None:
        """Update the transform from the hip to the knee. Uses pre-set angles and lengths."""
        self.hip2knee_transform = \
            np.array([[cos(self.hip_angle_rad), -sin(self.hip_angle_rad), 0.0, self.thigh_len_m*cos(self.hip_angle_rad)],
                      [sin(self.hip_angle_rad),  cos(self.hip_angle_rad), 0.0, self.thigh_len_m*sin(self.hip_angle_rad)],
                      [                    0.0,                      0.0, 1.0,                                      0.0],
                      [                    0.0,                      0.0, 0.0,                                      1.0]])

    def update_knee2foot_transform(self) -> None:
        """Update the transform from the knee to the foot. Uses pre-set angles and lengths."""
        self.knee2foot_transform = \
            np.array([[cos(self.knee_angle_rad), -sin(self.knee_angle_rad), 0.0, self.shin_len_m*cos(self.knee_angle_rad)],
                      [sin(self.knee_angle_rad),  cos(self.knee_angle_rad), 0.0, self.shin_len_m*sin(self.knee_angle_rad)],
                      [                     0.0,                       0.0, 1.0,                                      0.0],
                      [                     0.0,                       0.0, 0.0,                                      1.0]])

    def set_angles(self, coxa_angle_rad: float, hip_angle_rad: float, knee_angle_rad: float) -> None:
        """Set the angles of the leg and update transformation matrices as needed."""
        self.coxa_angle_rad = coxa_angle_rad
        self.hip_angle_rad = hip_angle_rad
        self.knee_angle_rad = knee_angle_rad
        # update tranformation matrices for joints
        self.update_body2coxa_transform()
        self.update_coxa2hip_transform()
        self.update_hip2knee_transform()
        self.update_knee2foot_transform()

    def set_transform_to_body(self, body2leg_transform: npt.NDArray) -> None:
        """Set transformation from body pose to leg pose.

        This transform is to the top of the leg (the hip joint)."""
        self.body2base_transform = body2leg_transform

    def get_transform_to_body(self) -> npt.NDArray:
        """Get transformation from body pose to leg pose"""
        return self.body2base_transform

    def set_foot_pose_in_local_coords(self, x: float, y: float, z: float) -> None:
        """Set position of the foot. Joint angles to achieve the position are calculated via inverse kinematics from the
        input coordinates (leg frame).
        """
        # Supporting variable D
        D = (x**2 + y**2 + z**2 - self.pelvis_len_m**2 - \
             self.thigh_len_m**2 - self.shin_len_m**2) / \
             (2*self.thigh_len_m*self.shin_len_m)
        # constrain magnitude of D to be less than 1 (otherwise we get sqrt errors)
        D = max(-1.0, min(D, 1.0))

        new_knee_angle = atan2(sqrt(1-D**2), D)

        # Secondary supporting variable
        D2 = x**2 + y**2 - self.pelvis_len_m**2
        D2 = max(0, D2)
        new_hip_angle = atan2(z, sqrt(D2)) - \
            atan2(self.shin_len_m*sin(new_knee_angle),
                  self.thigh_len_m + self.shin_len_m*cos(new_knee_angle))

        new_coxa_angle = atan2(y, x) + atan2(sqrt(D2), -self.pelvis_len_m)

        self.set_angles(new_coxa_angle, new_hip_angle, new_knee_angle)

    def set_foot_pose_in_body_coords(self, x: float, y: float, z: float) -> None:
        """Set position of the foot. Joint angles to achieve the position are calculated via inverse kinematics from the
        input coordinates (body frame).
        """
        # First invert the base2body transform
        # This is easy to do in parts, because it's just the product of inverse rotation and negative translation matrices
        # Also, inverse of an SO(3) matrix is equal to its own transpose.
        rotation = self.body2base_transform[0:3, 0:3].transpose()
        translation = -1 * self.body2base_transform[0:3, 3]

        homog_rotation = np.eye(4)
        homog_rotation[0:3, 0:3] = rotation
        homog_translation = np.eye(4)
        homog_translation[0:3, 3] = translation

        base2body = np.matmul(homog_rotation, homog_translation)

        # create the homogeneous foot position in the body coordinate system
        foot_pose_body = np.array([[x], [y], [z], [1.0]])

        # convert body coord foot pose to leg coordinates
        foot_pose = base2body.dot(foot_pose_body)
        # logging.get_logger(self._leg_name).info("pose in foot coords: {}, {}, {}, body coords: {}, {}, {}".format(foot_pose[0], foot_pose[1], foot_pose[2], x, y, z))

        # Now set the foot position in leg coordinates
        self.set_foot_pose_in_local_coords(
            foot_pose[0], foot_pose[1], foot_pose[2])

    def get_leg_points(self) -> npt.NDArray:
        """Get the coordinates for the four points that define leg pose.

            Point 1: body to hip joint location
            Point 2: hip to thigh joint location
            Point 3: thigh to ankle joint location
            Point 4: end of foot location

        The way we calculate this is equivalent to calculating the foot pose in body coordinates, but we keep
        track of intermediate variables.
        """
        p1 = self.body2base_transform[0:3, 3]

        transform_buildup = np.matmul(np.matmul(
            self.body2base_transform, self.body2coxa_transform), self.coxa2hip_transform)
        p2 = transform_buildup[0:3, 3]

        transform_buildup = np.matmul(
            transform_buildup, self.hip2knee_transform)
        p3 = transform_buildup[0:3, 3]

        transform_buildup = np.matmul(
            transform_buildup, self.knee2foot_transform)
        p4 = transform_buildup[0:3, 3]

        return np.vstack((p1, p2, p3, p4))

    def get_foot_pose_in_body_coords(self) -> npt.NDArray:
        """Return the coordinates of the foot's position in the body frame."""
        foot_pose = np.matmul(np.matmul(np.matmul(np.matmul(self.body2base_transform,
                                                            self.body2coxa_transform),
                                                  self.coxa2hip_transform),
                                        self.hip2knee_transform),
                              self.knee2foot_transform)
        return foot_pose[0:3, 3]

    def get_leg_angles(self) -> npt.NDArray:
        """Return leg angles (hip, knee, ankle)."""
        return np.array([self.coxa_angle_rad,
                         self.hip_angle_rad,
                         self.knee_angle_rad])


class SpotKinematics():
    def __init__(self, pelvis_len_m: float = 0.065, thigh_len_m: float = 0.105, shin_len_m: float = 0.132, body_width_m: float = 0.120, body_len_m: float = 0.180, body_pitch_rad: float = 0.0, body_roll_rad: float = 0.0, body_yaw_rad: float = 0.0, body_x_m: float = 0.0, body_y_m: float = 0.200, body_z_m: float = 0.0) -> None:
        """Initialize dimensions and pose of spot for use in motion calculations.
        """
        self.pelvis_len_m = pelvis_len_m
        self.thigh_len_m = thigh_len_m
        self.shin_len_m = shin_len_m
        self.body_width_m = body_width_m
        self.body_len_m = body_len_m
        self.neutral_height_m = thigh_len_m + shin_len_m

        # starting rotation and position of the body
        # We start with full extension legs so that we simplify initialization
        # The last step of this init method is set the actual body pose
        rot_homog = np.eye(4)
        trans_homog = np.eye(4)
        trans_homog[3, 0:3] = np.array([0.0, thigh_len_m + shin_len_m, 0.0])
        # construct body transform from angles and x,y,z pos
        self.t_body = np.matmul(trans_homog, rot_homog)

        right_leg_start_angles = np.array([0.0, 0.0, 0.0])
        left_leg_start_angles = np.array([0.0, 0.0, 0.0])

        # initialize legs with default 0 angles (standing)
        # Note: left/right is handled in servo cal (reversing angles so "forward" means the
        # same on both sides). All legs are treated identically in kinematics.
        self.legs = {
            "fl": LegKinematics(self.t_body2fl(), pelvis_len_m, thigh_len_m, shin_len_m, right_leg_start_angles, "fl"),
            "fr": LegKinematics(self.t_body2fr(), pelvis_len_m, thigh_len_m, shin_len_m, right_leg_start_angles, "fr"),
            "bl": LegKinematics(self.t_body2bl(), pelvis_len_m, thigh_len_m, shin_len_m, left_leg_start_angles, "bl"),
            "br": LegKinematics(self.t_body2br(), pelvis_len_m, thigh_len_m, shin_len_m, left_leg_start_angles, "br")
        }

        # now update body transform
        self.set_body_angles(body_pitch_rad, body_roll_rad, body_yaw_rad, body_y_m)

    def angles_to_SO2(self, pitch_rad, roll_rad, yaw_rad):
        roll = np.array([[1.0,           0.0,            0.0],
                         [0.0, cos(roll_rad), -sin(roll_rad)],
                         [0.0, sin(roll_rad),  cos(roll_rad)]])

        yaw = np.array([[cos(yaw_rad),  0.0, sin(yaw_rad)],
                        [         0.0,  1.0,          0.0],
                        [-sin(yaw_rad), 0.0, cos(yaw_rad)]])

        pitch = np.array([[cos(pitch_rad), -sin(pitch_rad), 0.0],
                          [sin(pitch_rad),  cos(pitch_rad), 0.0],
                          [           0.0,             0.0, 1.0]])
        rotation = np.matmul(np.matmul(roll, yaw), pitch)
        return rotation

    def get_body_to_leg_transform(self, angle, x_dist, z_dist) -> npt.NDArray:
        """Return the transform from the center of the body to a leg located at (angle, x, y) from center."""
        return np.array([[ cos(angle), 0.0, sin(angle), x_dist],
                         [        0.0, 1.0,        0.0,    0.0],
                         [-sin(angle), 0.0, cos(angle), z_dist],
                         [        0.0, 0.0,        0.0,    1.0]])

    def t_body2fl(self):
        """Return a transform from the body pose to the front-left leg origin."""
        return np.matmul(self.t_body,
                         self.get_body_to_leg_transform(pi/2, self.body_len_m/2, -self.body_width_m/2))

    def t_body2fr(self):
        """Return a transform from the body pose to the front-right leg origin."""
        return np.matmul(self.t_body,
                         self.get_body_to_leg_transform(pi/2, self.body_len_m/2, self.body_width_m/2))

    def t_body2bl(self):
        """Return a transform from the body pose to the back-left leg origin."""
        return np.matmul(self.t_body,
                         self.get_body_to_leg_transform(pi/2, -self.body_len_m/2, -self.body_width_m/2))

    def t_body2br(self):
        """Return a transform from the body pose to the back-right leg origin."""
        return np.matmul(self.t_body,
                         self.get_body_to_leg_transform(pi/2, -self.body_len_m/2, self.body_width_m/2))

    def get_joint_angles(self) -> npt.NDArray:
        """Gets the current joint angles for the entire robot.

        Joint angles are expected to be an array of:
            (flc, flh, flk;
             frc, frh, frk;
             blc, blh, blk;
             brc, brh, brk)
        """
        return np.vstack((self.legs["fl"].get_leg_angles(),
                          self.legs["fr"].get_leg_angles(),
                          self.legs["bl"].get_leg_angles(),
                          self.legs["br"].get_leg_angles()))

    def set_joint_angles(self, joint_angles: npt.NDArray) -> None:
        """Sets the current joint angles for the entire robot.

        The joints shall be an array of:
            (flc, flh, flk;
             frc, frh, frk;
             blc, blh, blk;
             brc, brh, brk)
        """
        self.legs["fl"].set_angles(
            joint_angles[0, 0], joint_angles[0, 1], joint_angles[0, 2])
        self.legs["fr"].set_angles(
            joint_angles[1, 0], joint_angles[1, 1], joint_angles[1, 2])
        self.legs["bl"].set_angles(
            joint_angles[2, 0], joint_angles[2, 1], joint_angles[2, 2])
        self.legs["br"].set_angles(
            joint_angles[3, 0], joint_angles[3, 1], joint_angles[3, 2])

    def get_foot_coords(self) -> npt.NDArray:
        """Return the current foot positions relative to body pose."""
        return np.vstack((self.legs["fl"].get_foot_pose_in_body_coords(),
                          self.legs["fr"].get_foot_pose_in_body_coords(),
                          self.legs["bl"].get_foot_pose_in_body_coords(),
                          self.legs["br"].get_foot_pose_in_body_coords()))

    def set_foot_coords(self, foot_coords: npt.NDArray) -> None:
        """Sets position of feet relative to body pose.

        Foot position should be a 4x3 array with columns as (x, y, z) positions and rows as the different legs.
        """
        self.legs["fl"].set_foot_pose_in_body_coords(
            foot_coords[0, 0], foot_coords[0, 1], foot_coords[0, 2])
        self.legs["fr"].set_foot_pose_in_body_coords(
            foot_coords[1, 0], foot_coords[1, 1], foot_coords[1, 2])
        self.legs["bl"].set_foot_pose_in_body_coords(
            foot_coords[2, 0], foot_coords[2, 1], foot_coords[2, 2])
        self.legs["br"].set_foot_pose_in_body_coords(
            foot_coords[3, 0], foot_coords[3, 1], foot_coords[3, 2])

    def set_body_transform(self, body_transform, body_height):
        """Set the body transform without changing the locations of the feet.

        This is equivalent to just changing the "lean" of the body.
        """
        feet_coords = self.get_foot_coords()

        self.t_body = body_transform

        self.legs["fl"].set_transform_to_body(self.t_body2fl())
        self.legs["fr"].set_transform_to_body(self.t_body2fr())
        self.legs["bl"].set_transform_to_body(self.t_body2bl())
        self.legs["br"].set_transform_to_body(self.t_body2br())

        feet_coords[:,1] += np.ones(4)*(self.neutral_height_m - body_height)

        self.set_foot_coords(feet_coords)

    def set_body_angles(self, body_pitch_rad: float = 0.0, body_roll_rad: float = 0.0, body_yaw_rad: float = 0.0, height = None) -> None:
        """Sets lean of body (pose) while leaving foot positions fixed.

        phi = roll = x, theta = pitch = z, psi = yaw = y

        TODO: we may want a getter for body angles
        """
        rotation = self.angles_to_SO2(
            body_pitch_rad, body_roll_rad, body_yaw_rad)
        new_transform = self.t_body
        new_transform[0:3, 0:3] = rotation

        if height is None:
            height = self.neutral_height_m
        self.set_body_transform(new_transform, height)
