#! /usr/bin/env python3

# ROS node to turn joystick msgs into Messages for Victor

from copy import deepcopy

import rclpy
from arm_utilities.listener import Listener
from arm_utilities.ros_helpers import joy_to_xbox
from numpy import clip
from rclpy.node import Node
from sensor_msgs.msg import Joy
from victor_hardware_interfaces.msg import Robotiq3FingerStatus, Robotiq3FingerCommand


def minus(xbox_lhs, xbox_rhs):
    """
    Returns a new Xbox_msg object with values lhs - rhs
    """
    xbox_diff = deepcopy(xbox_lhs)
    xbox_diff.A -= xbox_rhs.A
    xbox_diff.B -= xbox_rhs.B
    xbox_diff.X -= xbox_rhs.X
    xbox_diff.Y -= xbox_rhs.Y
    xbox_diff.LB -= xbox_rhs.LB
    xbox_diff.RB -= xbox_rhs.RB
    xbox_diff.back -= xbox_rhs.back
    xbox_diff.start -= xbox_rhs.start
    xbox_diff.power -= xbox_rhs.power
    xbox_diff.stick_button_left -= xbox_rhs.stick_button_left
    xbox_diff.stick_button_right -= xbox_rhs.stick_button_right
    xbox_diff.LH -= xbox_rhs.LH
    xbox_diff.LV -= xbox_rhs.LV
    xbox_diff.LT -= xbox_rhs.LT
    xbox_diff.RH -= xbox_rhs.RH
    xbox_diff.RV -= xbox_rhs.RV
    xbox_diff.RT -= xbox_rhs.RT
    xbox_diff.DH -= xbox_rhs.DH
    xbox_diff.DV -= xbox_rhs.DV
    return xbox_diff


class RobotiqGrippersJoystickNode(Node):
    def __init__(self):
        super().__init__("robotiq_grippers_joystick")
        self.output_throttle_period = 5.0

        self.gripper_status = {
            "right": Listener("right_arm/gripper_status", Robotiq3FingerStatus),
            "left": Listener("left_arm/gripper_status", Robotiq3FingerStatus)
        }

        self.gripper_command_publisher = {
            "right": self.create_publisher(Robotiq3FingerCommand, "right_arm/gripper_command", 10),
            "left": self.create_publisher(Robotiq3FingerCommand, "left_arm/gripper_command", 10)
        }

        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        self.prev_xbox_msg = None

    def joy_callback(self, joy_msg):
        """
        Assumes that we are using xboxdrv without mimic mode
        """
        xbox_msg = joy_to_xbox(joy_msg, xpad=False)
        if self.prev_xbox_msg is None:
            self.prev_xbox_msg = xbox_msg

        enable_finger_open_close_control = rospy.get_param("~enable_finger_open_close_control", True)
        enable_scissor_open_close_control = rospy.get_param("~enable_scissor_open_close_control", True)

        rospy.loginfo_throttle(self.output_throttle_period,
                               "Finger open close control enabled:  " + str(enable_finger_open_close_control))
        rospy.loginfo_throttle(self.output_throttle_period,
                               "Scissor open close control enabled: " + str(enable_scissor_open_close_control))

        if enable_finger_open_close_control:
            self.finger_open_close_callback(xbox_msg)

        if enable_scissor_open_close_control:
            self.scissor_open_close_callback(xbox_msg)

        self.prev_xbox_msg = xbox_msg

    def finger_open_close_callback(self, xbox_msg):
        """Open and close the gripper fingers"""
        xboxdiff = minus(xbox_msg, self.prev_xbox_msg)
        gripper_stop_dist = 0.05

        if xboxdiff.LT > 0:
            self.stop_gripper("left", gripper_stop_dist)

        elif xboxdiff.LB < 0:
            self.stop_gripper("left", -gripper_stop_dist)

        else:
            if xbox_msg.LT == -1:
                self.close_gripper("left")

            if xbox_msg.LB:
                self.open_gripper("left")

        if xboxdiff.RT > 0:
            self.stop_gripper("right", gripper_stop_dist)

        elif xboxdiff.RB < 0:
            self.stop_gripper("right", -gripper_stop_dist)

        else:
            if xbox_msg.RT == -1:
                self.close_gripper("right")

            if xbox_msg.RB:
                self.open_gripper("right")

    def scissor_open_close_callback(self, xbox_msg):
        """Open and close the scissors on the gripper"""
        xboxdiff = minus(xbox_msg, self.prev_xbox_msg)
        scissor_stop_dist = 0.05

        if xboxdiff.X < 0:
            self.stop_scissor("left", scissor_stop_dist)
        elif xboxdiff.Y < 0:
            self.stop_scissor("left", -scissor_stop_dist)
        else:
            if xbox_msg.X:
                self.close_scissor("left")

            if xbox_msg.Y:
                self.open_scissor("left")

        if xboxdiff.A < 0:
            self.stop_scissor("right", scissor_stop_dist)
        elif xboxdiff.B < 0:
            self.stop_scissor("right", -scissor_stop_dist)
        else:
            if xbox_msg.A:
                self.close_scissor("right")

            if xbox_msg.B:
                self.open_scissor("right")

    def stop_gripper(self, gripper_name, motion=0.0):
        """stops gripper fingers in current position + motion to allow for delay"""
        cur = self.gripper_status[gripper_name].get()
        finger_pos = [clip(cur.finger_a_status.position + motion, 0.0, 1.0),
                      clip(cur.finger_b_status.position + motion, 0.0, 1.0),
                      clip(cur.finger_c_status.position + motion, 0.0, 1.0)]

        self.set_gripper(gripper_name, finger_pos=finger_pos)

    def stop_scissor(self, gripper_name, motion=0.0):
        """stops gripper scissor in current position + motion to allow for delay"""
        cur = self.gripper_status[gripper_name].get()
        scissor_pos = cur.scissor_status.position + motion

        self.set_gripper(gripper_name, scissor_pos=scissor_pos)

    def close_gripper(self, gripper_name):
        self.set_gripper(gripper_name, finger_pos=(1.0, 1.0, 1.0))

    def open_gripper(self, gripper_name):
        self.set_gripper(gripper_name, finger_pos=(0.0, 0.0, 0.0))

    def close_scissor(self, gripper_name):
        self.set_gripper(gripper_name, scissor_pos=1.0)

    def open_scissor(self, gripper_name):
        self.set_gripper(gripper_name, scissor_pos=0.0)

    def set_gripper(self, gripper_name, finger_pos=None, scissor_pos=None):
        """
        Sets the gripper finger position, as well as the scissor

        Parameters:
        gripper_name  string  - "left" or "right"
        finger_pos    float[] - position values for fingers a,b,c. [0 to 1]
        scissor_pos   float   - position values for scissor. [0 to 1]
        """

        cur = self.gripper_status[gripper_name].get()
        cmd = self.default_gripper_command()

        # Set the finger position if commanded
        if finger_pos is not None:
            cmd.finger_a_command.position = finger_pos[0]
            cmd.finger_b_command.position = finger_pos[1]
            cmd.finger_c_command.position = finger_pos[2]
        else:
            cmd.finger_a_command.position = cur.finger_a_status.position_request
            cmd.finger_b_command.position = cur.finger_b_status.position_request
            cmd.finger_c_command.position = cur.finger_c_status.position_request

        # Set the scissor position if commanded
        if scissor_pos is not None:
            cmd.scissor_command.position = scissor_pos
        else:
            cmd.scissor_command.position = cur.scissor_status.position_request

        self.gripper_command_publisher[gripper_name].publish(cmd)

    def default_gripper_command(self):
        cmd = Robotiq3FingerCommand()
        cmd.finger_a_command.speed = 0.5
        cmd.finger_b_command.speed = 0.5
        cmd.finger_c_command.speed = 0.5
        cmd.scissor_command.speed = 1.0

        cmd.finger_a_command.force = 1.0
        cmd.finger_b_command.force = 1.0
        cmd.finger_c_command.force = 1.0
        cmd.scissor_command.force = 1.0

        cmd.scissor_command.position = 1.0
        return cmd


def main():
    rclpy.init()
    node = RobotiqGrippersJoystickNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
