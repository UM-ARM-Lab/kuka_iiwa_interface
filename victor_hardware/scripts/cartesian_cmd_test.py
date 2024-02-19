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

import tf2_ros

from rclpy.time import Time
from tf2_py import TransformException

from victor_hardware.victor import Victor
from victor_hardware_interfaces.msg import MotionCommand, ControlMode

import rclpy
from rclpy.node import Node


class CartesianCommandTestNode(Node):

    def __init__(self):
        super().__init__("victor_vr_teleop")

        self.victor = Victor(self)

        # create a TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Set up ros timer
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.victor.set_left_arm_control_mode(ControlMode.CARTESIAN_IMPEDANCE)

        self.t0 = None

    def timer_cb(self):
        if self.t0 is None:
            # try to get the current pose of the left gripper
            try:
                self.t0 = self.tf_buffer.lookup_transform('victor_left_palm', 'victor_left_arm_link0', Time())
                print(f"Got transform: {self.t0}")
            except TransformException as ex:
                self.get_logger().info(f'Could not transform: {ex}')
                return
        else:
            msg = MotionCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.control_mode.mode = ControlMode.CARTESIAN_IMPEDANCE

            msg.cartesian_pose.position.x = self.t0.transform.translation.x
            msg.cartesian_pose.position.y = self.t0.transform.translation.y
            msg.cartesian_pose.position.z = self.t0.transform.translation.z
            msg.cartesian_pose.orientation.x = self.t0.transform.rotation.x
            msg.cartesian_pose.orientation.y = self.t0.transform.rotation.y
            msg.cartesian_pose.orientation.z = self.t0.transform.rotation.z
            msg.cartesian_pose.orientation.w = self.t0.transform.rotation.w

            self.victor.left_arm_cmd_pub.publish(msg)


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
