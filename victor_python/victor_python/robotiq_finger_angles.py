from math import radians


def compute_finger_angles(control):
    """
    Returns the joint angles for the Robotiq 3-finger gripper based on the control value.

    Args:
        control: the control is from 0 to 1. 0 corresponds to fully open, 1 is fully closed.
    """
    g = control * 255
    max_angle = [70.0, 90.0, 43.0]
    min_3 = -55.0
    m1 = max_angle[0] / 140.0
    m2 = max_angle[1] / 100.0

    # http://motion.pratt.duke.edu/papers/IUCS-TR711-Franchi-gripper.pdf
    # Based on the relationship from the documentation, set each joint angle based on the "phase" of the motion
    if g <= 110.0:
        theta1 = m1 * g
        theta2 = 0
        theta3 = -m1 * g
    elif 110.0 < g <= 140.0:
        theta1 = m1 * g
        theta2 = 0
        theta3 = min_3
    elif 140.0 < g <= 240.0:
        theta1 = max_angle[0]
        theta2 = m2 * (g - 140)
        theta3 = min_3
    else:
        theta1 = max_angle[0]
        theta2 = max_angle[1]
        theta3 = min_3

    return [radians(theta1), radians(theta2), radians(theta3)]


def get_finger_angle_names(side: str, finger: str):
    """
    Returns the names of the finger joints for the specified side.

    Args:
        side: The side of the robot (left or right).
    """
    return [
        f"victor_{side}_{finger}_joint_1",
        f"victor_{side}_{finger}_joint_2",
        f"victor_{side}_{finger}_joint_3",
    ]

def get_scissor_joint_name(side: str, finger: str):
    return f"victor_{side}_palm_{finger}_joint"

def compute_scissor_angle(control):
    # 0 corresponds to fully open at -16 degrees, 1 is fully closed with at +10 degrees
    return radians(26.0 * control - 16.0)
