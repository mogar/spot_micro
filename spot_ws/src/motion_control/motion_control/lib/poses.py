
from spot_interfaces.msg import JointAngles


def get_standing_pose():
    stand_angles = JointAngles()
    # stand state is all 0 angles (spot micro is calibrated to standing
    return stand_angles


def get_sitting_pose():
    sit_angles = JointAngles()

    # Front Left Leg
    sit_angles.flc = 0.0
    sit_angles.flh = 20.0
    sit_angles.flk = 20.0
    # Front Right Leg
    sit_angles.frc = 0.0
    sit_angles.frh = 20.0
    sit_angles.frk = 20.0
    # Back Left Leg
    sit_angles.blc = 0.0
    sit_angles.blh = -45.0
    sit_angles.blk = 60.0
    # Back Right Leg
    sit_angles.brc = 0.0
    sit_angles.brh = -45.0
    sit_angles.brk = 60.0

    return sit_angles