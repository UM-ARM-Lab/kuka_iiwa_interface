#!/usr/bin/env python3
"""
This script receives teleop commands from the Unity VR system, and sends them to Victor.
Press and hold the grip button on the VR controller to start recording an episode.
Release the grip button to stop recording an episode.
Press the menu button to stop the script.
Press the trackpad to reset
Use the trigger to open and close the gripper, it's mapped to the open fraction of the gripper.
The motion will be relative in gripper frame to the pose of the gripper when you started recording.
see the vr_ros2_bridge repo for setup instructions.
"""

from copy import deepcopy
from math import sin

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from victor_python.victor import Victor


class CartesianCommandTestNode(Node):

    def __init__(self):
        super().__init__("victor_vr_teleop")

        self.victor = Victor(self)

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.initial_pose = None
        self.mode_set = False

        self.pub_idx = 0

    def timer_cb(self):
        if self.initial_pose is None:
            print("Waiting for initial status from left arm...")
            status = self.victor.left.motion_status.get()
            if status:
                self.initial_pose = status.commanded_cartesian_pose
        else:
            msg = deepcopy(self.initial_pose)
            delta = 0.04 * sin(self.pub_idx / 5)
            msg.cartesian_pose.position.x = self.initial_pose.position.x + delta

            self.victor.left.cartesian_cmd_pub.publish(msg)

            self.pub_idx += 1


def main():
    rclpy.init()

    node = CartesianCommandTestNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except SystemExit:
        pass

    rclpy.shutdown()
    print("Done!")


if __name__ == '__main__':
    main()
