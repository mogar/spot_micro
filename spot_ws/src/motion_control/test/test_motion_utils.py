import motion_control.lib.motion_utils as mu

from spot_interfaces.msg import JointAngles

def test_multi_joint_one_step_interp():
    joint_start = JointAngles()
    joint_end = JointAngles()

    joint_end.flc = 1.0
    joint_end.flh = 2.0
    joint_end.flk = 3.0
    joint_end.frc = 4.0
    joint_end.frh = 5.0
    joint_end.frk = 6.0
    joint_end.blc = 1.0
    joint_end.blh = 2.0
    joint_end.blk = 3.0
    joint_end.brc = 4.0
    joint_end.brh = 5.0
    joint_end.brk = 6.0

    ratio_increment = 0.1
    max_increment = ratio_increment*joint_end.brk

    next_joint_step = mu.multi_joint_one_step_interp(joint_start, joint_end, max_increment)

    joint_interp = JointAngles()
    joint_interp.flc = joint_end.flc*ratio_increment
    joint_interp.flh = joint_end.flh*ratio_increment
    joint_interp.flk = joint_end.flk*ratio_increment
    joint_interp.frc = joint_end.frc*ratio_increment
    joint_interp.frh = joint_end.frh*ratio_increment
    joint_interp.frk = joint_end.frk*ratio_increment
    joint_interp.blc = joint_end.blc*ratio_increment
    joint_interp.blh = joint_end.blh*ratio_increment
    joint_interp.blk = joint_end.blk*ratio_increment
    joint_interp.brc = joint_end.brc*ratio_increment
    joint_interp.brh = joint_end.brh*ratio_increment
    joint_interp.brk = joint_end.brk*ratio_increment

    assert mu.joint_angles_match(joint_interp, next_joint_step)
