"""

"""

from threading import Thread
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from victor_python.victor import Victor

def main():
    rclpy.init()
    node = Node("create_collision_scene")
    victor = Victor(node)
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin)
    spin_thread.start()

    res = victor.set_controller("impedance_controller")
    while True:
        # read current joint angles and set the current joint angles as the target joint angles
        current_joints = victor.get_left_joint_positions()

if __name__ == '__main__':
    main()