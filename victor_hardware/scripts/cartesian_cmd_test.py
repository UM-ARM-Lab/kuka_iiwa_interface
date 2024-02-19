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

import time
from pathlib import Path

import numpy as np

import rclpy
from victor_hardware.victor import Victor
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

class CartesianCommandTestNode(Node):

    def __init__(self):
        super().__init__("victor_vr_teleop")

        self.victor = Victor(self)

        # subscribe to TF

        # setup ros timer

    def timer_cb(self):
        pass

def main():
    rclpy.init()

    node = CartesianCommandTestNode()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass

    rclpy.shutdown()
    print("Done!")


if __name__ == '__main__':
    main()
