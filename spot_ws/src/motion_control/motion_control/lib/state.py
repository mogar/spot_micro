
from rclpy import logging

from spot_interfaces.msg import JointAngles
from geometry_msgs.msg import Twist
from spot_interfaces.msg import StateCmd

from motion_control.lib.walking import WalkManager
from motion_control.lib.motion_utils import one_step_interp
import motion_control.lib.poses as poses


class BaseState():
    """
    Base class for a state machine to handle motion (and stationary poses) of a 
    Spot Micro robot.
    """

    def __init__(self):
        pass

    def next_state_from_cmd(self, cmd, state_cmd):
        # no state transitions in base state, just return self
        return self

    def joint_angles_from_cmd(self, current_angles, cmd, state_cmd):
        # Base State does no action, so just return current angles unchanged
        return current_angles

class SitState(BaseState):
    """
    Class to handle the sitting state of a spot micro. Includes functionality
    to transition from a non-sitting pose to a sitting pose.
    """

    def __init__(self):
        self._sit_angles = poses.get_sitting_pose()

    def next_state_from_cmd(self, cmd, state_cmd):
        if state_cmd.sit == 0:
            logging.get_logger("SitState").info("transition to stand")
            return StandState()
        return self

    def joint_angles_from_cmd(self, current_angles, cmd, state_cmd, max_angle_delta):
        target_angles = JointAngles()
        target_angles.flc = one_step_interp(current_angles.flc, self._sit_angles.flc, max_angle_delta)
        target_angles.flh = one_step_interp(current_angles.flh, self._sit_angles.flh, max_angle_delta)
        target_angles.flk = one_step_interp(current_angles.flk, self._sit_angles.flk, max_angle_delta)
        target_angles.frc = one_step_interp(current_angles.frc, self._sit_angles.frc, max_angle_delta)
        target_angles.frh = one_step_interp(current_angles.frh, self._sit_angles.frh, max_angle_delta)
        target_angles.frk = one_step_interp(current_angles.frk, self._sit_angles.frk, max_angle_delta)
        target_angles.blc = one_step_interp(current_angles.blc, self._sit_angles.blc, max_angle_delta)
        target_angles.blh = one_step_interp(current_angles.blh, self._sit_angles.blh, max_angle_delta)
        target_angles.blk = one_step_interp(current_angles.blk, self._sit_angles.blk, max_angle_delta)
        target_angles.brc = one_step_interp(current_angles.brc, self._sit_angles.brc, max_angle_delta)
        target_angles.brh = one_step_interp(current_angles.brh, self._sit_angles.brh, max_angle_delta)
        target_angles.brk = one_step_interp(current_angles.brk, self._sit_angles.brk, max_angle_delta)
        return target_angles

class StandState(BaseState):
    """
    Class to handle the standing state of a spot micro. Includes functionality
    to transition from a non-standing pose to a standing pose.
    """
    def __init__(self):
        self._stand_angles = poses.get_standing_pose()

    def next_state_from_cmd(self, cmd, state_cmd):
        if state_cmd.sit != 0:
            logging.get_logger("StandState").info("transition to sit")
            return SitState()
        # TODO: check twist_cmd for high velocity and move to walking
        return self

    def joint_angles_from_cmd(self, current_angles, cmd, state_cmd, max_angle_delta):
        # TODO: look at z-twist to see if we should raise/lower, etc.
        target_angles = JointAngles()
        target_angles.flc = one_step_interp(current_angles.flc, self._stand_angles.flc, max_angle_delta)
        target_angles.flh = one_step_interp(current_angles.flh, self._stand_angles.flh, max_angle_delta)
        target_angles.flk = one_step_interp(current_angles.flk, self._stand_angles.flk, max_angle_delta)
        target_angles.frc = one_step_interp(current_angles.frc, self._stand_angles.frc, max_angle_delta)
        target_angles.frh = one_step_interp(current_angles.frh, self._stand_angles.frh, max_angle_delta)
        target_angles.frk = one_step_interp(current_angles.frk, self._stand_angles.frk, max_angle_delta)
        target_angles.blc = one_step_interp(current_angles.blc, self._stand_angles.blc, max_angle_delta)
        target_angles.blh = one_step_interp(current_angles.blh, self._stand_angles.blh, max_angle_delta)
        target_angles.blk = one_step_interp(current_angles.blk, self._stand_angles.blk, max_angle_delta)
        target_angles.brc = one_step_interp(current_angles.brc, self._stand_angles.brc, max_angle_delta)
        target_angles.brh = one_step_interp(current_angles.brh, self._stand_angles.brh, max_angle_delta)
        target_angles.brk = one_step_interp(current_angles.brk, self._stand_angles.brk, max_angle_delta)
        return target_angles

class WalkState(BaseState):
    """
    Class to handle the walking state of a spot micro. Mostly serves as a wrapper for the WalkManager class.
    """
    def __init__(self):
        self._walk_mgr = WalkManager()

    def next_state_from_cmd(self, cmd, state_cmd):
        if self._walk_mgr.is_standing(cmd):
            # switch to stand state if we're stationary and in stand stance
            return StandState()
        return self

    def joint_angles_from_cmd(self, current_angles, cmd, state_cmd, max_angle_delta):
        return self._walk_mgr(current_angles, cmd, max_angle_delta)
