

class WalkManager():
    def __init__(self):
        # Leg IDs
        # FL = 0
        # FR = 1
        # BL = 2
        # BR = 3
        self._moving_leg = 0

        # walking states
        # transition to walk: 0
        # walking: 1
        # transitioning to stand: 2
        self._walking_state = 0

    def is_standing(self, cmd):
        # TODO: check speed in cmd to see if we should be transitioning to stand
        return True

    def new_joint_angles(self, current_angles, cmd, max_angle_delta):
        # TODO: walking math
        return current_angles