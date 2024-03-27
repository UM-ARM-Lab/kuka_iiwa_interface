"""
Moves the final joint of the right arm by +/- 0.5 radians.
"""
from copy import copy

import numpy as np
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_state import RobotState

from arm_robots.robot import load_moveitpy

ROBOT_NAME = "victor"

# This has to match a group in the SRDF file describing the robot.
GROUP_NAME = "right_arm"

JOINT_7_DELTA_DEG = 30


def main():
    moveitpy, moveit_config = load_moveitpy(ROBOT_NAME)
    planning_component = moveitpy.get_planning_component(GROUP_NAME)

    # planning_component.set_start_state("...")
    planning_component.set_start_state_to_current_state()

    psm = moveitpy.get_planning_scene_monitor()

    scene: PlanningScene
    with psm.read_only() as scene:
        current_robot_state: RobotState = scene.current_state
        initial_joint_positions = current_robot_state.get_joint_group_positions(GROUP_NAME)

    goal_joint_positions = copy(initial_joint_positions)
    if goal_joint_positions[-1] < 0:
        goal_joint_positions[-1] += np.deg2rad(JOINT_7_DELTA_DEG)
    else:
        goal_joint_positions[-1] -= np.deg2rad(JOINT_7_DELTA_DEG)

    # The configuration name is a group_state in the SRDF.
    # planning_component.set_goal_state(configuration_name="impedance_switch")

    # We need to pass a RobotState to the `set_goal_state` method. Here, we grab that.
    robot_model = moveitpy.get_robot_model()
    robot_state = RobotState(robot_model)

    # Change robot state to the desired configuration.
    robot_state.set_joint_group_positions(GROUP_NAME, goal_joint_positions)

    planning_component.set_goal_state(robot_state=robot_state)

    plan_result = planning_component.plan()

    if plan_result:
        robot_trajectory = plan_result.trajectory
        moveitpy.execute(robot_trajectory, controllers=[])
    else:
        raise RuntimeError("Planning failed!")


if __name__ == "__main__":
    main()
