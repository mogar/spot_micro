
from geometry_msgs.msg import Twist

from spot_interfaces.msg import JointAngles

import motion_control.lib.motion_utils as motion_utils
import motion_control.lib.poses as poses

import copy

class WalkManager():
    def __init__(self) -> None:
        # walking states
        # transition to walk: 0
        # walking: 1
        # transitioning to stand: 2
        # standing: 3
        self._walking_state = 0

        # track whether we are swinging a leg or shifting center of gravity
        # swing up:   0
        # swing down: 1
        # shift:      2
        self._swing_phase = 0

        # Leg IDs
        # FL = 0
        # FR = 1
        # BL = 2
        # BR = 3
        self._moving_leg = 0
        self._num_legs = 4

        # SpotKinematics defaults to correct size and standing pose
        self._kinematics = SpotKinematics()
        # we use this stand foot pos variable as a constant reference
        self._stand_foot_pos = self._kinematics.get_foot_coords()
        # this is what gets updated for the pose we are aiming at
        self._target_foot_pos = self._kinematics.get_foot_coords()

        # We use a triangular stepping motion for each leg in sequence. We start at
        # the starting position, interpolate motion up to the high leg position, then
        # interpolate down to the stop leg position. After the leg is in the stop position,
        # the bot shifts its center of gravity to leave the next leg in the start position.

        # foot position when leg is up
        self._leg_up_height = 0.03
        
        # foot position at end of step
        self._leg_stride = 0.03


    def is_standing(self, cmd: Twist) -> bool:
        # TODO: more robust speed checking
        if (self._walking_state == 3) and (cmd.linear.x < 2):
            return True
        return False

    def new_joint_angles(self, current_angles: JointAngles, cmd: Twist, max_angle_delta: int) -> JointAngles:
        if self._walking_state == 2:
            return self.to_stand_angles(current_angles, max_angle_delta)
        else: # TODO: assuming walking and transition to walk can use same controller
            return self.walking_angles(current_angles, cmd, max_angle_delta)

    def to_stand_angles(self, current_angles: JointAngles, max_angle_delta: int) -> JointAngles:
        # set angles to 0 and feet to standing pose
        self._kinematics.set_body_angles(body_pitch_rad = 0.0, body_roll_rad = 0.0, body_yaw_rad = 0.0)
        self._kinematics.set_foot_coords(self._stand_foot_pos)
        target_angles = self._kinematics.get_joint_angles()
        target_joints = mu.np_array_to_joint_angles(target_angles)

        if motion_utils.joint_angles_match(current_angles, target_joints):
            self.walking_state = 3
        else:
            target_joints = mu.multi_joint_one_step_interp(current_angles, target_joints, max_angle_delta)
        return target_joints

    def walking_angles(self, current_angles: JointAngles, cmd: Twist, max_angle_delta: int) -> JointAngles:
        # update leg phase
        self.update_phase(current_angles)

        # TODO: calculate speed from max_angle_delta and cmd
        speed = max_angle_delta

        return self.get_next_joint_angles(current_angles, speed)

    def update_phase(self, current_angles: JointAngles) -> None:
        # NOTE: currently assuming kinematics is set correctly for the current phase
        target_angles = self._kinematics.get_joint_angles()
        target_joints = mu.np_array_to_joint_angles(target_angles)

        # calculate whether we're swinging or shifting
        if self._swing_phase == 0:
            # we're swinging up
            if motion_utils.joint_angles_match(current_angles, target_joints):
                # leg has achieved raised pose
                self._swing_phase = 1
                # TODO: change stride somehow??
                self._target_foot_pos = get_walking_foot_poses(self._moving_leg, self._leg_up_height, self._leg_stride)
                self._kinematics.set_foot_coords(self._target_foot_pos)
        if self._swing_phase == 1:
            # we're swinging down
            if motion_utils.joint_angles_match(current_angles, target_joints):
                # leg has achieved lowered pose
                self._swing_phase = 2
                # TODO: change stride somehow??
                self._target_foot_pos = get_walking_foot_poses(self._moving_leg, 0.0, self._leg_stride)
                self._kinematics.set_foot_coords(self._target_foot_pos)
        else:
            # we're shifting the body
            if motion_utils.joint_angles_match(current_angles, target_joints):
                # body shift is done
                self._moving_leg += 1
                if self._moving_leg >= self._num_legs:
                    self._moving_leg = 0
                self._swing_phase = 0
                self._target_foot_pos = get_walking_foot_poses(self._moving_leg, 0.0, self._leg_stride)
                self._kinematics.set_foot_coords(self._target_foot_pos)

    def get_next_joint_angles(self, current_angles: JointAngles, angle_speed: int) -> JointAngles:
        # NOTE: assuming that kinematics is already set for the target pose
        target_angles = self._kinematics.get_joint_angles()
        target_joints = mu.np_array_to_joint_angles(target_angles)

        target_joints = mu.multi_joint_one_step_interp(current_angles, target_joints, max_angle_delta)
        return target_joints

    def get_walking_foot_poses(self, active_leg_id: int, active_foot_height: float, stride: float) -> npt.NDArray:
        new_foot_pos = copy.copy(self._stand_foot_pos)
        new_foot_pos[active_leg_id, 1] += active_foot_height

        # TODO: handle stride

        return new_foot_pos
