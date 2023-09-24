
from spot_interfaces.msg import JointAngles


def get_standing_pose():
    stand_angles = JointAngles()
    # stand state is all 0 angles (spot micro is calibrated to standing
    return stand_angles


def get_sitting_pose():
    sit_angles = JointAngles()

    # Front Left Leg
    sit_angles.fls = 0.0
    sit_angles.fle = 20.0
    sit_angles.flw = 20.0
    # Front Right Leg
    sit_angles.frs = 0.0
    sit_angles.fre = 20.0
    sit_angles.frw = 20.0
    # Back Left Leg
    sit_angles.bls = 0.0
    sit_angles.ble = -45.0
    sit_angles.blw = 60.0
    # Back Right Leg
    sit_angles.brs = 0.0
    sit_angles.bre = -45.0
    sit_angles.brw = 60.0

    return sit_angles