
from rclpy import logging

from spot_interfaces.msg import JointAngles
from geometry_msgs.msg import Twist
from spot_interfaces.msg import StateCmd

from motion_control.lib.walking import WalkManager
from motion_control.lib.motion_utils import one_step_interp


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
        self._sit_angles = JointAngles()
        # Front Left Leg
        self._sit_angles.fls = 0.0
        self._sit_angles.fle = 20.0
        self._sit_angles.flw = 20.0
        # Front Right Leg
        self._sit_angles.frs = 0.0
        self._sit_angles.fre = 20.0
        self._sit_angles.frw = 20.0
        # Back Left Leg
        self._sit_angles.bls = 0.0
        self._sit_angles.ble = -45.0
        self._sit_angles.blw = 60.0
        # Back Right Leg
        self._sit_angles.brs = 0.0
        self._sit_angles.bre = -45.0
        self._sit_angles.brw = 60.0

    def next_state_from_cmd(self, cmd, state_cmd):
        if state_cmd.sit == 0:
            logging.get_logger("SitState").info("transition to stand")
            return StandState()
        return self

    def joint_angles_from_cmd(self, current_angles, cmd, state_cmd, max_angle_delta):
        target_angles = JointAngles()
        target_angles.fls = one_step_interp(current_angles.fls, self._sit_angles.fls, max_angle_delta)
        target_angles.fle = one_step_interp(current_angles.fle, self._sit_angles.fle, max_angle_delta)
        target_angles.flw = one_step_interp(current_angles.flw, self._sit_angles.flw, max_angle_delta)
        target_angles.frs = one_step_interp(current_angles.frs, self._sit_angles.frs, max_angle_delta)
        target_angles.fre = one_step_interp(current_angles.fre, self._sit_angles.fre, max_angle_delta)
        target_angles.frw = one_step_interp(current_angles.frw, self._sit_angles.frw, max_angle_delta)
        target_angles.bls = one_step_interp(current_angles.bls, self._sit_angles.bls, max_angle_delta)
        target_angles.ble = one_step_interp(current_angles.ble, self._sit_angles.ble, max_angle_delta)
        target_angles.blw = one_step_interp(current_angles.blw, self._sit_angles.blw, max_angle_delta)
        target_angles.brs = one_step_interp(current_angles.brs, self._sit_angles.brs, max_angle_delta)
        target_angles.bre = one_step_interp(current_angles.bre, self._sit_angles.bre, max_angle_delta)
        target_angles.brw = one_step_interp(current_angles.brw, self._sit_angles.brw, max_angle_delta)
        return target_angles

class StandState(BaseState):
    """
    Class to handle the standing state of a spot micro. Includes functionality
    to transition from a non-standing pose to a standing pose.
    """
    def __init__(self):
        # stand state is all 0 angles (spot micro is calibrated to standing)
        self._stand_angles = JointAngles()

    def next_state_from_cmd(self, cmd, state_cmd):
        if state_cmd.sit != 0:
            logging.get_logger("StandState").info("transition to sit")
            return SitState()
        # TODO: check twist_cmd for high velocity and move to walking
        return self

    def joint_angles_from_cmd(self, current_angles, cmd, state_cmd, max_angle_delta):
        # TODO: look at z-twist to see if we should raise/lower, etc.
        target_angles = JointAngles()
        target_angles.fls = one_step_interp(current_angles.fls, self._stand_angles.fls, max_angle_delta)
        target_angles.fle = one_step_interp(current_angles.fle, self._stand_angles.fle, max_angle_delta)
        target_angles.flw = one_step_interp(current_angles.flw, self._stand_angles.flw, max_angle_delta)
        target_angles.frs = one_step_interp(current_angles.frs, self._stand_angles.frs, max_angle_delta)
        target_angles.fre = one_step_interp(current_angles.fre, self._stand_angles.fre, max_angle_delta)
        target_angles.frw = one_step_interp(current_angles.frw, self._stand_angles.frw, max_angle_delta)
        target_angles.bls = one_step_interp(current_angles.bls, self._stand_angles.bls, max_angle_delta)
        target_angles.ble = one_step_interp(current_angles.ble, self._stand_angles.ble, max_angle_delta)
        target_angles.blw = one_step_interp(current_angles.blw, self._stand_angles.blw, max_angle_delta)
        target_angles.brs = one_step_interp(current_angles.brs, self._stand_angles.brs, max_angle_delta)
        target_angles.bre = one_step_interp(current_angles.bre, self._stand_angles.bre, max_angle_delta)
        target_angles.brw = one_step_interp(current_angles.brw, self._stand_angles.brw, max_angle_delta)
        return target_angles

    class StandState(BaseState):
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
