
from geometry_msgs.msg import Twist

from spot_interfaces.msg import JointAngles

import motion_control.lib.motion_utils as mu
import motion_control.lib.poses as poses
from motion_control.lib.spot_kinematics import SpotKinematics

import copy
import numpy as np

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
        self._leg_stride = 0.1


    def is_standing(self, cmd: Twist) -> bool:
        # TODO: more robust speed checking
        if (abs(cmd.linear.x) < 0.5):
            if (self._walking_state == 3):
                return True
            self._walking_state = 2
        return False

    def new_joint_angles(self, current_angles: JointAngles, cmd: Twist, max_angle_delta: int) -> JointAngles:
        if self._walking_state == 2 or self._walking_state == 3:
            return self.to_stand_angles(current_angles, max_angle_delta)
        else:
            return self.walking_angles(current_angles, cmd, max_angle_delta)

    def to_stand_angles(self, current_angles: JointAngles, max_angle_delta: int) -> JointAngles:
        target_angles = self._kinematics.get_joint_angles()
        target_joints = mu.np_array_to_joint_angles(target_angles)

        # check if we're done with the current phase
        if mu.joint_angles_match(current_angles, target_joints):
            if self._walking_state != 3:
                # assume we're already standing
                self._walking_state = 3
                legs_checked = 0
                while legs_checked <= self._num_legs:
                    # make sure each leg is in the right position for standing
                    if np.all(self._target_foot_pos[self._moving_leg,:] != self._stand_foot_pos[self._moving_leg,:]):
                        # this leg isn't in the right position yet
                        if self._target_foot_pos[self._moving_leg,1] > self._stand_foot_pos[self._moving_leg,1]:
                            # we already have the foot up, so now put the foot down
                            self._target_foot_pos[self._moving_leg,:] = self._stand_foot_pos[self._moving_leg,:]
                        else:
                            # the target foot is too low, raise it up so we can reposition the whole leg
                            self._target_foot_pos[self._moving_leg,1] = self._stand_foot_pos[self._moving_leg,1] + self._leg_up_height
                        # we're not ready to stand yet
                        self._walking_state = 2
                        break
                    else:
                        # that leg is good, move to the next one
                        self._moving_leg += 1
                        if self._moving_leg >= self._num_legs:
                            self._moving_leg = 0
                        legs_checked += 1
            else:
                # we're already in the standing state
                self._target_foot_pos = self._stand_foot_pos

        self._kinematics.set_foot_coords(self._target_foot_pos)
        return self.get_next_joint_angles(current_angles, max_angle_delta)

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

        # check if we're done with the current phase
        if mu.joint_angles_match(current_angles, target_joints):
            # calculate whether we're swinging or shifting
            if self._swing_phase == 0:
                # we're done swinging up, now swing down
                self._swing_phase = 1
                # set target position for swinging down
                self._target_foot_pos[self._moving_leg, 1] = self._stand_foot_pos[self._moving_leg, 1]
                self._target_foot_pos[self._moving_leg, 0] += self._leg_stride/2
            elif self._swing_phase == 1:
                # leg has achieved lowered pose, now shift
                self._swing_phase = 2
                # set target position shifting (all legs)
                # we shift 4 times in a cycle, so we div by 4 to make a whole stride in one cycle
                self._target_foot_pos[0, 0] -= self._leg_stride/4
                self._target_foot_pos[1, 0] -= self._leg_stride/4
                self._target_foot_pos[2, 0] -= self._leg_stride/4
                self._target_foot_pos[3, 0] -= self._leg_stride/4
            else:
                # body shift is done, change legs and move the next up
                self._moving_leg += 1
                if self._moving_leg >= self._num_legs:
                    self._moving_leg = 0
                self._swing_phase = 0

                # set target position for swinging up
                self._target_foot_pos[self._moving_leg, 1] += self._leg_up_height
                self._target_foot_pos[self._moving_leg, 0] += self._leg_stride/2
            # update foot coordinates now that we've calculated them
            self._kinematics.set_foot_coords(self._target_foot_pos)

    def get_next_joint_angles(self, current_angles: JointAngles, max_angle_delta: int) -> JointAngles:
        # NOTE: assuming that kinematics is already set for the target pose
        target_angles = self._kinematics.get_joint_angles()
        target_joints = mu.np_array_to_joint_angles(target_angles)

        target_joints = mu.multi_joint_one_step_interp(current_angles, target_joints, max_angle_delta)
        return target_joints

