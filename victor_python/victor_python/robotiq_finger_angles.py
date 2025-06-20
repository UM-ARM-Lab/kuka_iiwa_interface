from math import radians, degrees


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


class RobotiqROSTranslateCMD:
    # This is a singular version since no parallelization is needed in actual ROS
    # There is a parallel version in the isaacsim implementation of robot_tool_2025S
    # constants from the technical doc
    _TH1_MAX = 70.0       # degrees
    _TH2_MAX = 90.0       # degrees
    _TH3_MIN = -55.0      # degrees
    _M1 = _TH1_MAX / 140.0
    _M2 = _TH2_MAX / 100.0
    _SCISSOR_MIN = -16.0  # degrees
    _SCISSOR_MAX =  10.0  # degrees
    _SCISSOR_RANGE = _SCISSOR_MAX - _SCISSOR_MIN

    @classmethod
    def forward(cls, ros_cmds):
        a_cmd, b_cmd, c_cmd, scissor_cmd = ros_cmds

        # enforce any scissor‐vs‐finger constraint on raw inputs
        scissor_cmd = cls._solve_scissor_constraints(
            a_cmd, b_cmd, c_cmd, scissor_cmd
        )

        # map into actual joint angles (radians)
        scissor_mapped = cls._forward_scissor_cmd(scissor_cmd)
        a_mapped = cls._forward_finger_cmd(a_cmd)
        b_mapped = cls._forward_finger_cmd(b_cmd)
        c_mapped = cls._forward_finger_cmd(c_cmd)

        # reorder for IsaacSim [scissor_a, scissor_b, a_root, c_root, b_root, ...]
        root_cmds = [a_mapped[0], c_mapped[0], b_mapped[0]]
        mid_cmds  = [a_mapped[1], c_mapped[1], b_mapped[1]]
        tip_cmds  = [a_mapped[2], c_mapped[2], b_mapped[2]]

        return scissor_mapped + root_cmds + mid_cmds + tip_cmds

    @classmethod
    def inverse(cls, isaacsim_cmds):
        # split out scissor and fingers
        sc_a, sc_b = isaacsim_cmds[0], isaacsim_cmds[1]
        sc_cmd = cls._inverse_scissor_cmd([sc_a, sc_b])

        # reorder fingers back to a,b,c
        a_j = [isaacsim_cmds[2], isaacsim_cmds[5], isaacsim_cmds[8]]
        c_j = [isaacsim_cmds[3], isaacsim_cmds[6], isaacsim_cmds[9]]
        b_j = [isaacsim_cmds[4], isaacsim_cmds[7], isaacsim_cmds[10]]

        a_cmd = cls._inverse_finger_cmd(a_j)
        b_cmd = cls._inverse_finger_cmd(b_j)
        c_cmd = cls._inverse_finger_cmd(c_j)

        return [a_cmd, b_cmd, c_cmd, sc_cmd]

    @classmethod
    def _forward_scissor_cmd(cls, control):
        # 0→–16°, 1→+10°
        c = max(0.0, min(control, 1.0))
        θ = cls._SCISSOR_MIN + cls._SCISSOR_RANGE * c
        θ_rad = radians(θ)
        return [θ_rad, -θ_rad]

    @classmethod
    def _inverse_scissor_cmd(cls, isaac_scissor_cmds):
        θ_rad = isaac_scissor_cmds[0]
        θ_deg = degrees(θ_rad)
        c = (θ_deg - cls._SCISSOR_MIN) / cls._SCISSOR_RANGE
        return max(0.0, min(c, 1.0))

    @classmethod
    def _forward_finger_cmd(cls, control):
        # simply delegate to the spec‐correct mapping
        return compute_finger_angles(control)

    @classmethod
    def _inverse_finger_cmd(cls, isaac_finger_cmds):
        # isaac_finger_cmds are [θ1,θ2,θ3] in radians
        th1, th2, _ = isaac_finger_cmds
        d1 = degrees(th1)
        d2 = degrees(th2)

        # Phase 3 (fully saturated) ⇒ return 1.0
        if d1 >= cls._TH1_MAX and d2 >= cls._TH2_MAX:
            return 1.0

        # Phase 2 or 3 (where θ2>0):  g = θ2/m2 + 140
        if d2 > 0.0:
            g = d2 / cls._M2 + 140.0
        # Phase 1 or 1′ (θ2=0):  g = θ1/m1
        else:
            g = d1 / cls._M1

        c = g / 255.0
        return max(0.0, min(c, 1.0))

    @classmethod
    def _solve_scissor_constraints(cls, a_raw, b_raw, c_raw, scissor_raw):
        # keep your existing constraint logic (thresholds are empirical)
        needs = (a_raw + b_raw) > 1.2 or (a_raw + c_raw) > 1.2
        return min(scissor_raw, 0.5) if needs else scissor_raw
