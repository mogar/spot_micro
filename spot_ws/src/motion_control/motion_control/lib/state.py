
from rclpy import logging

from spot_interfaces.msg import JointAngles
from geometry_msgs.msg import Twist
from spot_interfaces.msg import StateCmd

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
    def __init__(self):
        self._sit_angles = JointAngles()
        # TODO: fill in _sit_angles with what a sit should actually look like

    def next_state_from_cmd(self, cmd, state_cmd):
        if state_cmd.sit == 0:
            logging.get_logger("SitState").info("transition to stand")
            return StandState()
        return self

    def joint_angles_from_cmd(self, current_angles, cmd, state_cmd):
        # TODO: if current_angles isn't sit_angles, move
        return current_angles

class StandState(BaseState):
    def __init__(self):
        self._stand_angles = JointAngles()
        # TODO: fill in _stand_angles with what a stand should actually look like

    def next_state_from_cmd(self, cmd, state_cmd):
        if state_cmd.sit != 0:
            logging.get_logger("StandState").info("transition to sit")
            return SitState()
        return self

    def joint_angles_from_cmd(self, current_angles, cmd, state_cmd):
        # TODO: if current_angles isn't stand_angles, move
        return current_angles