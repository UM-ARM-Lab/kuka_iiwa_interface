import time
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
    victor.add_collision_box("wall_back", position=(-0.4, -0., 1.15), quat_xyzw=(0, 0, 0, 1),
                             size=(0.2, 2, 1.7))
    victor.add_collision_box("wall_left", position=(0.4, 1.1,  1), quat_xyzw=(0, 0, 0, 1),
                             size=(1.6, 0.1, 2))
    victor.add_collision_box("wall_right", position=(0.4, -1.1,  1), quat_xyzw=(0, 0, 0, 1),
                             size=(1.6, 0.1, 2))
    # sleep for some time to wait for the collision objects to be added
    time.sleep(3)
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()
    exit(0)




if __name__ == '__main__':
    main()
