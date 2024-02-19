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

from math import cos, sin
from copy import deepcopy

from victor_hardware.victor import Victor
from victor_hardware_interfaces.msg import MotionCommand, ControlMode

import rclpy
from rclpy.node import Node


class CartesianCommandTestNode(Node):

    def __init__(self):
        super().__init__("victor_vr_teleop")

        self.victor = Victor(self)

        self.timer = self.create_timer(0.2, self.timer_cb)

        self.initial_pose = None
        self.mode_set = False

        self.pub_idx = 0

    def timer_cb(self):
        # if not self.mode_set:
        #     self.mode_set = True
        #     self.victor.set_left_arm_control_mode(ControlMode.CARTESIAN_IMPEDANCE)

        if self.initial_pose is None:
            status = self.victor.left_arm_status_listener.get()
            if status:
                self.initial_pose = status.commanded_cartesian_pose
        else:
            msg = MotionCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base"
            msg.control_mode.mode = ControlMode.CARTESIAN_IMPEDANCE

            msg.cartesian_pose = deepcopy(self.initial_pose)
            dz = 0.1 * sin(self.pub_idx / 5)
            msg.cartesian_pose.position.z = self.initial_pose.position.z + dz

            self.victor.left_arm_cmd_pub.publish(msg)

            self.pub_idx += 1


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
