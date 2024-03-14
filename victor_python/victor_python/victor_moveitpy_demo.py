"""

TODO: make this move a delta from the current config so it's safer

"""
from arm_robots.robot import load_moveitpy
from moveit.core.robot_state import RobotState
import numpy as np

ROBOT_NAME = "victor"

# This has to match a group in the SRDF file describing the robot.
GROUP_NAME = "right_arm"

GOAL_JOINT_CONFIG = np.asarray(
        (0.724, 0.451, 0.940, -1.425, 0.472, 0.777, -0.809))

def main():
    moveitpy, moveit_config = load_moveitpy(ROBOT_NAME)
    planning_component = moveitpy.get_planning_component(GROUP_NAME)

    # planning_component.set_start_state("...")
    planning_component.set_start_state_to_current_state()

    # The configuration name is a group_state in the SRDF.
    # planning_component.set_goal_state(configuration_name="impedance_switch")

    # We need to pass a RobotState to the `set_goal_state` method. Here, we grab that.
    robot_model = moveitpy.get_robot_model()
    robot_state = RobotState(robot_model)

    # Change robot state to the desired configuration.
    robot_state.set_joint_group_positions(GROUP_NAME, GOAL_JOINT_CONFIG)

    planning_component.set_goal_state(robot_state=robot_state)

    plan_result = planning_component.plan()

    if plan_result:
        robot_trajectory = plan_result.trajectory
        moveitpy.execute(robot_trajectory, controllers=[])
    else:
        raise RuntimeError("Planning failed!")


if __name__ == "__main__":
    main()
