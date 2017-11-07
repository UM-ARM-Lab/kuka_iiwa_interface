#! /usr/bin/env python

# ROS node to turn joystick msgs into Messages for Victor

import rospy
from arc_utilities import ros_helpers
from victor_hardware_interface.msg import Robotiq3FingerStatus, Robotiq3FingerCommand
from sensor_msgs.msg import Joy


class VictorJoystick:
    def __init__(self):
        self.gripper_status = \
            {"right": ros_helpers.Listener("right_arm/gripper_status", Robotiq3FingerStatus),
             "left": ros_helpers.Listener("left_arm/gripper_status", Robotiq3FingerStatus)}

        self.gripper_command_publisher = \
            {"right": rospy.Publisher("right_arm/gripper_command", Robotiq3FingerCommand,
                                      queue_size = 1),
             "left": rospy.Publisher("left_arm/gripper_command", Robotiq3FingerCommand,
                                     queue_size = 1)}

        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

    def joy_callback(self, joy):
        A, B, X, Y, LB, RB, back, start, power, stick_button_left, stick_button_right, \
            dleft, dright, dup, ddown = joy.buttons
        left_hor, left_vert, LT, right_hor, right_vert,  RT, d_hor, d_vert = joy.axes

        if LT == -1:
            self.close_gripper("left")

        if LB:
            self.open_gripper("left")

        if RT == -1:
            self.close_gripper("right")

        if RB:
            self.open_gripper("right")

    def close_gripper(self, gripper_name, **kwargs):
        self.set_gripper(gripper_name, (1.0, 1.0, 1.0), **kwargs)

    def open_gripper(self, gripper_name, **kwargs):
        self.set_gripper(gripper_name, (0.0, 0.0, 0.0), **kwargs)

    def set_gripper(self, gripper_name, pos, blocking=True, continuous=False):
        """
        Sets the gripper finger position. Does not set scissor position

        Parameters:
        gripper_name  string  - "left" or "right"
        pos       float[]   - position values for fingers a,b,c. [0 to 1]
        """
        
        a = pos[0]
        b = pos[1]
        c = pos[2]
        cur = self.gripper_status[gripper_name].get()
        cmd = self.default_gripper_command()
        
        cmd.finger_a_command.position = a
        cmd.finger_b_command.position = b
        cmd.finger_c_command.position = c
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
    rospy.init_node('xbox_control')
    VJ = VictorJoystick()
    rospy.spin()


if __name__ == "__main__":
    main()


