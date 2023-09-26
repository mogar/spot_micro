
import numpy as np
import numpy.typing as npt

class SpotLeg():
    def __init__(self):
        # TODO:
        pass

class SpotKinematics():
    def __init__(self, hip_len_cm = 2, thigh_len_cm = 3, shin_len_cm = 4, body_width_cm = 5, body_len_cm = 6, body_pitch_rad = 0, body_roll_rad = 0, body_yaw_rad = 0):
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

    def get_joint_angles(self) -> npt.NDArray:
        """Gets the current joint angles for the entire robot.
        """
        # TODO: return joint angles
        return np.array([0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0])

    def set_joint_angles(self, joint_angles: npt.NDArray) -> None:
        """Sets the current joint angles for the entire robot.
        """
        # TODO:
        pass

    def get_foot_coords(self) -> npt.NDArray:
        """Return the current foot positions relative to center of mass.
        """
        # TODO:
        return np.array([0])

    def set_foot_coords(self, foot_coords: npt.NDArray) -> None:
        """Sets position of feet relative to body center of mass.
        """
        # TODO:
        # input is 4x3 array of foot positions (x,y,z) relative to center of mass
        pass

    def get_body_lean(self) -> npt.NDArray:
        """Return the current lean of the body.
        """
        # TODO:
        return np.array([0])

    def set_body_lean(self, body_lean: npt.NDArray) -> None:
        """Sets lean of body (pose) while leaving foot positions fixed.
        """
        # TODO:
        pass

    