
from geometry_msgs.msg import Twist

from spot_interfaces.msg import JointAngles

import motion_control.lib.motion_utils as motion_utils
import motion_control.lib.poses as poses


class WalkManager():
    def __init__(self) -> None:
        # walking states
        # transition to walk: 0
        # walking: 1
        # transitioning to stand: 2
        # standing: 3
        self._walking_state = 0

        # track whether we are swinging a leg or shifting center of gravity
        self._swing_not_shift = True

        # Leg IDs
        # FL = 0
        # FR = 1
        # BL = 2
        # BR = 3
        self._moving_leg = 0
        self._num_legs = 4

        # We use a triangular stepping motion for each leg in sequence. We start at
        # the starting position, interpolate motion up to the high leg position, then
        # interpolate down to the stop leg position. After the leg is in the stop position,
        # the bot shifts its center of gravity to leave the next leg in the start position.
        # TODO: start leg angles
        # high leg angles
        self._high_leg_coxa = 0.0 # TODO:
        self._high_leg_hip = 5.0 # TODO:
        self._high_leg_knee = 20.0 # TODO:
        # stop leg angles
        self._stop_leg_coxa = 0.0 # TODO:
        self._stop_leg_hip = 20.0 # TODO:
        self._stop_leg_knee = -20.0 # TODO:


    def is_standing(self, cmd: Twist) -> bool:
        # TODO: more robust speed checking
        if (self._walking_state == 3) and (cmd.linear.y < 2):
            return True
        return False

    def new_joint_angles(self, current_angles: JointAngles, cmd: Twist, max_angle_delta: int) -> JointAngles:
        if self._walking_state == 2:
            return self.to_stand_angles(current_angles, max_angle_delta)
        else: # TODO: assuming walking and transition to walk can use same controller
            return self.walking_angles(current_angles, cmd, max_angle_delta)

    def to_stand_angles(self, current_angles: JointAngles, max_angle_delta: int) -> JointAngles:
        # TODO: calculate angles to return to standing

        if motion_utils.joint_angles_match(current_angles, poses.get_standing_pose()):
            self.walking_state = 3
        return current_angles

    def walking_angles(self, current_angles: JointAngles, cmd: Twist, max_angle_delta: int) -> JointAngles:
        # update leg phase
        self.update_phase(current_angles)

        # TODO: calculate speed from max_angle_delta and cmd
        speed = max_angle_delta

        if self._swing_not_shift:
            return self.triangular_interp_angles(current_angles, speed)
        else:
            return self.shift_center_of_gravity_angles(current_angles, speed)

    def update_phase(self, current_angles: JointAngles) -> None:
        # TODO: check if current angles shows we're at the end of the phase
        # foot on the ground, coxa forward
        # maybe have a "done leg pose" we're targeting

        # TODO: calculate whether we're swinging or shifting
        if self._swing_not_shift:
            # TODO: check that swing is in final leg pose
            self._swing_not_shift = False
        else:
            # TODO: check that shift is done
            self._moving_leg += 1
            if self._moving_leg >= self._num_legs:
                self._moving_leg = 0
            self._swing_not_shift = True

    def triangular_interp_angles(self, current_angles: JointAngles, angle_speed: int) -> JointAngles:
        active_leg_angles = motion_utils.get_leg_angles_as_np_array(current_angles, self._moving_leg)

        if active_leg_angles[0] <= self._high_leg_coxa:
            # ascending: leg angles interp up to top
            active_leg_angles[0] = motion_utils.one_step_interp(active_leg_angles[0], self._high_leg_coxa, angle_speed)
            active_leg_angles[1] = motion_utils.one_step_interp(active_leg_angles[1], self._high_leg_hip, angle_speed)
            active_leg_angles[2] = motion_utils.one_step_interp(active_leg_angles[2], self._high_leg_knee, angle_speed)
        else:
            # descending: leg angles interp down to stop
            active_leg_angles[0] = motion_utils.one_step_interp(active_leg_angles[0], self._stop_leg_coxa, angle_speed)
            active_leg_angles[1] = motion_utils.one_step_interp(active_leg_angles[1], self._stop_leg_hip, angle_speed)
            active_leg_angles[2] = motion_utils.one_step_interp(active_leg_angles[2], self._stop_leg_knee, angle_speed)

        new_angles = copy.copy(current_angles)
        motion_utils.set_leg_angles_in_joint_angles(new_angles, active_leg_angles, self._moving_leg)
        return new_angles

    def shift_center_of_gravity_angles(self, current_angles: JointAngles, angle_speed: int) -> JointAngles:
        # TODO:
        return current_angles