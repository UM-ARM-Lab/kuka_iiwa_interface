import os

import numpy as np
from moveit.core.planning_interface import MotionPlanResponse
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.node import Node
from victor_python.victor import Victor


def main():
    # this node isn't actually spun or used!
    rclpy.init()
    node = Node('test_moveitpyk')
    victor = Victor(node)

    moveit_config = (
        MoveItConfigsBuilder(robot_name="victor", package_name="victor_moveit_config")
        # .robot_description(file_path="config/panda.urdf.xacro")
        # .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("victor_moveit_config"),
                "config",
                "moveit_cpp.yaml"
            )
        )
        .to_moveit_configs()
    ).to_dict()

    moveitpy_robot = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
    left_arm = moveitpy_robot.get_planning_component('left_arm')

    # instantiate a RobotState instance using the current robot model
    robot_model = moveitpy_robot.get_robot_model()
    start_state = RobotState(robot_model)
    start_state.set_to_random_positions()

    goal_state = RobotState(robot_model)
    goal_state.set_to_random_positions()

    left_arm.set_start_state(robot_state=start_state)
    left_arm.set_goal_state(robot_state=goal_state)

    plan_result: MotionPlanResponse = left_arm.plan()

    keys = list(filter(lambda k: 'arm' in k, start_state.joint_positions.keys()))
    start_position_list = np.array([start_state.joint_positions[k] for k in keys])
    goal_position_list = np.array([goal_state.joint_positions[k] for k in keys])

    np.set_printoptions(precision=3, suppress=True, linewidth=150)
    print("Start state", start_position_list)
    print("Goal state", goal_position_list)
    print("Plan result:")
    print(f"Error code: {plan_result.error_code}")
    print(f"Duration: {plan_result.trajectory.duration}")


if __name__ == '__main__':
    main()
