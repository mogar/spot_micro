
from spot_interfaces.msg import JointAngles
from geometry_msgs.msg import Twist


class BaseState():
    """
    Base class for a state machine to handle motion (and stationary poses) of a 
    Spot Micro robot.
    """

    def __init__(self):
        pass

    def next_state_from_cmd(self, cmd):
        # no state transitions in base state, just return self
        return self

    def joint_angles_from_cmd(self, current_angles, cmd):
        # Base State does no action, so just return current angles unchanged
        return current_angles

class SitState(BaseState):
    def __init__(self):
        self._sit_angles = JointAngles()
        # TODO: fill in _sit_angles with what a sit should actually look like

    def next_state_from_cmd(self, cmd):
        # TODO: transition to stand based on command
        return self

    def joint_angles_from_cmd(self, current_angles, cmd):
        # TODO: if current_angles isn't sit_angles, move
        return current_angles

