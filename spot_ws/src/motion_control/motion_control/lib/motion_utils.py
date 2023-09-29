

import numpy as np
import numpy.typing as npt
from spot_interfaces.msg import JointAngles


def get_leg_angles_as_np_array(joints: JointAngles, leg_id: int) -> npt.NDArray:
    leg_angles = np.zeros(3)
    if leg_id == 0:
        # FL
        leg_angles[0] = joints.flc
        leg_angles[1] = joints.flh
        leg_angles[2] = joints.flk
    elif leg_id == 1:
        # FR
        leg_angles[0] = joints.frc
        leg_angles[1] = joints.frh
        leg_angles[2] = joints.frk
    elif leg_id == 2:
        # BL
        leg_angles[0] = joints.blc
        leg_angles[1] = joints.blh
        leg_angles[2] = joints.blk
    else:
        # BR
        leg_angles[0] = joints.brc
        leg_angles[1] = joints.brh
        leg_angles[2] = joints.brk
    return leg_angles

def set_leg_angles_in_joint_angles(joints: JointAngles, leg_angles: npt.NDArray, leg_id: int):
    if leg_angles.shape != (3,):
        # wrong leg angles shape, make no changes
        return

    if leg_id == 0:
        # FL
        joints.flc = leg_angles[0]
        joints.flh = leg_angles[1]
        joints.flk = leg_angles[2]
    elif leg_id == 1:
        # FR
        joints.frc = leg_angles[0]
        joints.frh = leg_angles[1]
        joints.frk = leg_angles[2]
    elif leg_id == 2:
        # BL
        joints.blc = leg_angles[0]
        joints.blh = leg_angles[1]
        joints.blk = leg_angles[2]
    else:
        # BR
        joints.brc = leg_angles[0]
        joints.brh = leg_angles[1]
        joints.brk = leg_angles[2]

def joint_angles_to_np_array(joints: JointAngles) -> npt.NDArray:
    """Convert joint angles represented as a JointAngles object into a numpy array.
    """
    angles = np.zeros(12) # 3 joints for each of 4 legs
    angles[0]  = joints.flc
    angles[1]  = joints.flh
    angles[2]  = joints.flk
    angles[3]  = joints.frc
    angles[4]  = joints.frh
    angles[5]  = joints.frk
    angles[6]  = joints.blc
    angles[7]  = joints.blh
    angles[8]  = joints.blk
    angles[9]  = joints.brc
    angles[10] = joints.brh
    angles[11] = joints.brk
    return angles

def np_array_to_joint_angles(array: npt.NDArray) -> JointAngles:
    """Convert jcint angles represented as a numpy array into a JointAngles object.
    """
    joints = JointAngles()
    if array.shape != (12,):
        # it's not a valid joint angles array, so just return all zeros as default
        return joints

    joints.flc = array[0]
    joints.flh = array[1]
    joints.flk = array[2]
    joints.frc = array[3]
    joints.frh = array[4]
    joints.frk = array[5]
    joints.blc = array[6]
    joints.blh = array[7]
    joints.blk = array[8]
    joints.brc = array[9]
    joints.brh = array[10]
    joints.brk = array[11]
    return joints

def joint_angles_match(joints_a: JointAngles, joints_b: JointAngles, tolerance_deg: float = 0.01) -> bool:
    """Return true if the joint angles in A are within tolerance of being equal to joint angles in B.
    """
    angles_a = joint_angles_to_np_array(joints_a)
    angles_b = joint_angles_to_np_array(joints_b)

    abs_diff = np.abs(angles_a - angles_b)

    return np.max(abs_diff) <= tolerance_deg

def multi_joint_one_step_interp(current_joints: JointAngles, target_joints: JointAngles, max_angle_delta: float) -> JointAngles:
    """Interpolate a joint position for Spot Micro based on current joint position and target joint position.

    Given a target position that differs in multiple joints from current, this method will find an interplated joint step
    that, when iterated, will lead all joints to land in the target on the same step (approximately).
    """
    # determine desired angle deltas for each joint
    current_angles_arr = joint_angles_to_np_array(current_joints)
    target_angles_arr = joint_angles_to_np_array(target_joints)
    angle_deltas = target_angles_arr - current_angles_arr

    # determine max desired change
    max_delta_mag = np.max(np.abs(angle_deltas))

    # constrain max desired change to max allowable
    max_step = min(max_delta_mag, max_angle_delta)

    # calculate ratio of max allowable to max desired, and adjust all angles based on this one ratio
    ratio = max_step / max_delta_mag
    angle_deltas = angle_deltas * ratio

    # add allowable angle deltas to current angles and convert back to JointAngles
    interp_angles = np_array_to_joint_angles(current_angles_arr + angle_deltas)
    return interp_angles

def one_step_interp(current_angle: float, target_angle: float, max_angle_delta: float) -> float:
    """Given a starting angle and an ending angle, determine an interpolated angle that is a maximum of
    `max_angle_delta` away from the current_angle.
    """
    max_positive_increment = max_angle_delta
    max_negative_increment = -1 * max_positive_increment

    delta_angle = target_angle - current_angle
    # clamp
    delta_angle = max(max_negative_increment, min(delta_angle, max_positive_increment))

    return current_angle + delta_angle
