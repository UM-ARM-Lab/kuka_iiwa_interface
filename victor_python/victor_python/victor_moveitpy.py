from arm_robots.robot import load_moveitpy
from victor_python.victor import Victor


class VictorMoveItPy:

    def __init__(self, victor: Victor):
        self.victor = victor
        self.moveitpy, self.moveit_config = load_moveitpy("victor")
        self.robot_model = self.moveitpy.get_robot_model()
