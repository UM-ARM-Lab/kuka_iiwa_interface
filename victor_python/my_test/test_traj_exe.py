from threading import Thread

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arm_utilities.transformation_helper import get_vec7_from_transform
from victor_python.victor import Victor
import sys

sys.path.append("/home/zixuanh/UMD")
from visualization.plot import plot_seg_fig, plot_pointclouds


def main():
    rclpy.init()
    node = Node("create_collision_scene")
    victor = Victor(node)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin)
    spin_thread.start()
    victor.clear_all_collision_objects()
    victor.add_collision_box("wall_back", position=(-0.4, -0., 1.15), quat_xyzw=(0, 0, 0, 1),
                             size=(0.2, 2, 1.7))
    victor.add_collision_box("wall_left", position=(0.4, 1.1, 1), quat_xyzw=(0, 0, 0, 1),
                             size=(1.6, 0.1, 2))
    victor.add_collision_box("wall_right", position=(0.4, -1.1, 1), quat_xyzw=(0, 0, 0, 1),
                             size=(1.6, 0.1, 2))
    # victor.add_collision_box("table", position=(0.7, -0., 0.49), quat_xyzw=(0, 0, 0, 1),
    #                          size=(0.9, 2, 0.3))
    victor.add_collision_box("table", position=(0.7, -0., 0.22), quat_xyzw=(0, 0, 0, 1),
                             size=(0.9, 2, 0.3))
    # victor.plan_to_pose("right_arm", "victor_right_tool0",
    #                     [0.64171, -0.62015, 1.1226, 0.7063, -0.1136, -0.68346,0.1453])
    # victor.plan_to_pose("left_arm", "victor_left_tool0",
    #                     [0.9883, 0.71313, 1.2721, -0.49767, -0.45559, 0.50329, 0.53987])

    res = victor.set_controller("joint_impedance_trajectory_controller")
    # old one
    # init_joints = [0.4972, 1.8683, 1.7862, 1.6785, -0.365, -0.292, 0.706]
    # init_pose = [0.44124, 0.094164, 0.78189, -0.5042, 0.46525, 0.49146, 0.53647]
    # init_pose = [0.44124, 0.094164, 0.52189, -0.5042, 0.46525, 0.49146, 0.53647]
    # v1
    init_joints = [1.81142560129673, -0.9190716461362816, 0.5205683046766082, 1.5782800558085164, 0.4801717476191806,
                   1.133958745559351, -2.2186526794434287]
    init_pose = [0.44124, -0.14164, 0.62189, -0.5042, 0.46525, 0.49146, 0.53647]
    # v2
    # init_pose = [0.44124, -0.0, 0.66189, -0.5042, 0.46525, 0.49146, 0.53647]
    # init_joints = [-1.4246661547111514,
    #                1.4717439207289167,
    #                -1.972077804912963,
    #                1.8651240122600667,
    #                0.092925435801626,
    #                1.8824929034403572,
    #                -2.7047007423155445]
    # v3
    # init_pose = [0.44124, -0.0, 0.51189, -0.5042, 0.46525, 0.49146, 0.53647]
    # init_joints = [-0.018376569895234566,
    #                1.669754738877708,
    #                1.5776807468810934,
    #                1.3918677575804785,
    #                -0.5718917326758477,
    #                -0.07594806425548087,
    #                0.6756722518012801]
    victor.plan_to_joint_config(init_joints, "right_arm")
    # victor.plan_to_pose(init_pose, "right_arm", "victor_right_tool0")
    print(victor.get_joint_cmd_dict())
    res = victor.set_controller("impedance_controller")

    ideal_poses = []
    actual_poses = []
    current_pose = victor.get_link_pose("victor_right_tool0")
    ideal_pos = np.array(get_vec7_from_transform(current_pose)[:3])
    delta = np.array([
        [0, -0.4, 0],
        [0.1, 0, 0],
        [0, 0.4, 0],
        [0.1, 0, 0],
    ])
    for i in range(5):
        for x in delta:
            tool_pose = victor.get_link_pose("victor_right_tool0")
            ideal_pos = ideal_pos + x
            tool_pose.translation.x = ideal_pos[0]
            tool_pose.translation.y = ideal_pos[1]
            tool_pose.translation.z = ideal_pos[2]

            ideal_poses.append(ideal_pos)
            victor.move_to_pose("right_arm", tool_pose)
            new_pose = victor.get_link_pose("victor_right_tool0")
            new_pos = get_vec7_from_transform(new_pose)[:3]
            actual_poses.append(new_pos)
            print("Execution error ", np.linalg.norm(new_pos - ideal_pos))
    plot_pointclouds([np.stack(ideal_poses), np.stack(actual_poses)]).show()
    tool_pose = victor.get_link_pose("victor_right_tool0")
    tool_pose.translation.x += 0.1
    victor.move_to_pose("right_arm", tool_pose)
    # test cartesian mode
    motion_status = victor.left.motion_status.get()
    print(motion_status.measured_cartesian_pose_abc)
    print(motion_status.commanded_cartesian_pose_abc)
    print(motion_status.measured_cartesian_pose)
    print(motion_status.commanded_cartesian_pose)


if __name__ == '__main__':
    main()