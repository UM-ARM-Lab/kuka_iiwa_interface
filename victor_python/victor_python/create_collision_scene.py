from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


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
    victor.add_collision_box("wall_back", position=[-0.4, -0., 1.15], quat_xyzw=[0, 0, 0, 1],
                             size=[0.2, 2, 1.7])
    victor.add_collision_box("wall_left", position=[0.4, 1.1,  1], quat_xyzw=[0, 0, 0, 1],
                             size=[1.6, 0.1, 2])
    victor.add_collision_box("wall_right", position=[0.4, -1.1,  1], quat_xyzw=[0, 0, 0, 1],
                             size=[1.6, 0.1, 2])
    # res = victor.set_controller("joint_impedance_trajectory_controller")

if __name__ == '__main__':
    main()
