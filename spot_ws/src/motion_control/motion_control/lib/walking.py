
from motion_control.lib.motion_utils import one_step_interp


class WalkManager():
    def __init__(self):
        # walking states
        # transition to walk: 0
        # walking: 1
        # transitioning to stand: 2
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
        self._high_leg_shoulder = 0.0 # TODO:
        self._high_leg_elbow = 5.0 # TODO:
        self._high_leg_wrist = 20.0 # TODO:
        # stop leg angles
        self._stop_leg_shoulder = 0.0 # TODO:
        self._stop_leg_elbow = 20.0 # TODO:
        self._stop_leg_wrist = -20.0 # TODO:


    def is_standing(self, cmd):
        # TODO: check speed in cmd to see if we should be transitioning to stand
        return True

    def new_joint_angles(self, current_angles, cmd, max_angle_delta):
        if self._walking_state == 2:
            return self.to_stand_angles(current_angles, max_angle_delta)
        else: # TODO: assuming walking and transition to walk can use same controller
            return self.walking_angles(current_angles, cmd, max_angle_delta)

    def to_stand_angles(self, current_angles, max_angle_delta):
        # TODO: calculate angles to return to standing
        return current_angles

    def walking_angles(self, current_angles, cmd, max_angle_delta):
        # update leg phase
        self.update_phase(current_angles)

        # TODO: calculate speed from max_angle_delta and cmd
        speed = max_angle_delta

        if self._swing_not_shift:
            return self.triangular_interp_angles(current_angles, speed)
        else:
            return self.shift_center_of_gravity_angles(current_angles, speed)

    def update_phase(self, current_angles):
        # TODO: calculate whether we're swinging or shifting
        self._swing_not_shift = True

        # TODO: check if current angles shows we're at the end of the phase
        # foot on the ground, shoulder forward
        # maybe have a "done leg pose" we're targeting

        self._moving_leg += 1
        if self._moving_leg >= self._num_legs:
            self._moving_leg = 0


    def triangular_interp_angles(self, current_angles, angle_speed):
        if self._moving_leg == 0: # FL
            active_shoulder = current_angles.fls
            active_elbow = current_angles.fle
            active_wrist = current_angles.flw
        if self._moving_leg == 1: # FR
            active_shoulder = current_angles.frs
            active_elbow = current_angles.fre
            active_wrist = current_angles.frw
        if self._moving_leg == 2: # BL
            active_shoulder = current_angles.bls
            active_elbow = current_angles.ble
            active_wrist = current_angles.blw
        else: # BR
            active_shoulder = current_angles.brs
            active_elbow = current_angles.bre
            active_wrist = current_angles.brw

        if active_shoulder <= self._high_leg_shoulder:
            # ascending: leg angles interp up to top
            active_shoulder = one_step_interp(active_shoulder, self._high_leg_shoulder, angle_speed)
            active_elbow = one_step_interp(active_elbow, self._high_leg_elbow, angle_speed)
            active_wrist = one_step_interp(active_wrist, self._high_leg_wrist, angle_speed)
        else:
            # descending: leg angles interp down to stop
            active_shoulder = one_step_interp(active_shoulder, self._stop_leg_shoulder, angle_speed)
            active_elbow = one_step_interp(active_elbow, self._stop_leg_elbow, angle_speed)
            active_wrist = one_step_interp(active_wrist, self._stop_leg_wrist, angle_speed)

        new_angles = copy.copy(current_angles)
        if self._moving_leg == 0: # FL
            new_angles.fls = active_shoulder
            new_angles.fle = active_elbow
            new_angles.flw = active_wrist
        if self._moving_leg == 1: # FR
            new_angles.frs = active_shoulder
            new_angles.fre = active_elbow
            new_angles.frw = active_wrist
        if self._moving_leg == 2: # BL
            new_angles.bls = active_shoulder
            new_angles.ble = active_elbow
            new_angles.blw = active_wrist
        else: # BR
            new_angles.brs = active_shoulder
            new_angles.bre = active_elbow
            new_angles.brw = active_wrist

        return new_angles

    def shift_center_of_gravity_angles(self, current_angles, angle_speed):
        # TODO:
        return current_angles