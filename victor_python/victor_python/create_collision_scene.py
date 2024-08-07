from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arm_utilities.transformation_helper import get_vec7_from_transform
from victor_python.victor import Victor


def main():
    rclpy.init()
    node = Node("create_collision_scene")
    victor = Victor(node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin)
    spin_thread.start()
    victor.clear_all_collision_objects()
    # victor.plan_to_pose("right_arm", "victor_right_tool0",
    #                     [0.64171, -0.62015, 1.1226, 0.7063, -0.1136, -0.68346,0.1453])
    # victor.plan_to_pose("left_arm", "victor_left_tool0",
    #                     [0.9883, 0.71313, 1.2721, -0.49767, -0.45559, 0.50329, 0.53987])
    victor.add_collision_box("wall_back", position=(-0.4, -0., 1.15), quat_xyzw=(0, 0, 0, 1),
                             size=(0.2, 2, 1.7))
    victor.add_collision_box("wall_left", position=(0.4, 1.1,  1), quat_xyzw=(0, 0, 0, 1),
                             size=(1.6, 0.1, 2))
    victor.add_collision_box("wall_right", position=(0.4, -1.1,  1), quat_xyzw=(0, 0, 0, 1),
                             size=(1.6, 0.1, 2))
    res = victor.set_controller("impedance_controller")
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
