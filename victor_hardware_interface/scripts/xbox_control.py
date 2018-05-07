#! /usr/bin/env python

# ROS node to turn joystick msgs into Messages for Victor

import rospy
from arc_utilities import ros_helpers
from victor_hardware_interface.msg import Robotiq3FingerStatus, Robotiq3FingerCommand
from sensor_msgs.msg import Joy
from copy import deepcopy

class XboxJoy:
    def __init__(self, joy_msg):
        A, B, X, Y, LB, RB, back, start, power, stick_button_left, stick_button_right, \
            dleft, dright, dup, ddown = joy_msg.buttons
        left_hor, left_vert, LT, right_hor, right_vert,  RT, d_hor, d_vert = joy_msg.axes
        self.A = A
        self.B = B
        self.X = X
        self.Y = Y
        self.LB = LB
        self.RB = RB
        self.back = back
        self.start = start
        self.power = power
        self.stick_button_left = stick_button_left
        self.stick_button_right = stick_button_right
        self.dleft = dleft
        self.dright = dright
        self.dup = dup
        self.ddown = ddown
        self.left_hor = left_hor
        self.left_vert = left_vert
        self.LT = LT
        self.right_hor = right_hor
        self.right_vert = right_vert
        self.RT =  RT
        self.d_hor = d_hor
        self.d_vert = d_vert

    def minus(self, other_joy):
        new_joy = deepcopy(self)
        new_joy.A -= other_joy.A
        new_joy.B -= other_joy.B
        new_joy.X -= other_joy.X
        new_joy.Y -= other_joy.Y
        new_joy.LB -= other_joy.LB
        new_joy.RB -= other_joy.RB
        new_joy.back -= other_joy.back
        new_joy.start -= other_joy.start
        new_joy.power -= other_joy.power
        new_joy.stick_button_left -= other_joy.stick_button_left
        new_joy.stick_button_right -= other_joy.stick_button_right
        new_joy.dleft -= other_joy.dleft
        new_joy.dright -= other_joy.dright
        new_joy.dup -= other_joy.dup
        new_joy.ddown -= other_joy.ddown
        new_joy.left_hor -= other_joy.left_hor
        new_joy.left_vert -= other_joy.left_vert
        new_joy.LT -= other_joy.LT
        new_joy.right_hor -= other_joy.right_hor
        new_joy.right_vert -= other_joy.right_vert
        new_joy.RT -= other_joy. RT
        new_joy.d_hor -= other_joy.d_hor
        new_joy.d_vert -= other_joy.d_vert
        return new_joy
        





class VictorJoystick:
    def __init__(self):
        self.output_throttle_period = 5.0

        self.gripper_status = \
            {"right": ros_helpers.Listener("right_arm/gripper_status", Robotiq3FingerStatus),
             "left": ros_helpers.Listener("left_arm/gripper_status", Robotiq3FingerStatus)}

        self.gripper_command_publisher = \
            {"right": rospy.Publisher("right_arm/gripper_command", Robotiq3FingerCommand,
                                      queue_size = 1),
             "left": rospy.Publisher("left_arm/gripper_command", Robotiq3FingerCommand,
                                     queue_size = 1)}

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

        self.prev_joy = None

        

    def joy_callback(self, joy_msg):
        joy = XboxJoy(joy_msg)
        if(self.prev_joy is None):
            self.prev_joy = joy

        enable_finger_open_close_control = rospy.get_param("~enable_finger_open_close_control", True)
        enable_scissor_open_close_control = rospy.get_param("~enable_scissor_open_close_control", True)

        rospy.loginfo_throttle(self.output_throttle_period,
                               "Finger open close control enabled:  " + str(enable_finger_open_close_control))
        rospy.loginfo_throttle(self.output_throttle_period,
                               "Scissor open close control enabled: " + str(enable_scissor_open_close_control))

        if enable_finger_open_close_control:
            self.finger_open_close_callback(joy)

        if enable_scissor_open_close_control:
            self.scissor_open_close_callback(joy)

        self.prev_joy = joy

    def finger_open_close_callback(self, joy):
        # Open and close the gripper fingers
        joydiff = joy.minus(self.prev_joy)
        gripper_stop_dist = 0.05

        if joydiff.LT > 0:
            self.stop_gripper("left", gripper_stop_dist)
            
        elif joydiff.LB < 0:
            self.stop_gripper("left", -gripper_stop_dist)
            
        else:
            if joy.LT == -1:
                self.close_gripper("left")

            if joy.LB:
                self.open_gripper("left")

        if joydiff.RT > 0:
            self.stop_gripper("right", gripper_stop_dist)

        elif joydiff.RB < 0:
            self.stop_gripper("right", -gripper_stop_dist)

        else:
            if joy.RT == -1:
                self.close_gripper("right")

            if joy.RB:
                self.open_gripper("right")

    def scissor_open_close_callback(self, joy):
        # Open and close the scissors on the gripper
        joydiff = joy.minus(self.prev_joy)
        scissor_stop_dist = 0.05

        if joydiff.X < 0:
            self.stop_scissor("left", scissor_stop_dist)
        elif joydiff.Y < 0:
            self.stop_scissor("left", -scissor_stop_dist)
        else:
            if joy.X:
                self.close_scissor("left")

            if joy.Y:
                self.open_scissor("left")

        if joydiff.A < 0:
            self.stop_scissor("right", scissor_stop_dist)
        elif joydiff.B < 0:
            self.stop_scissor("right", -scissor_stop_dist)
        else:
            if joy.A:
                self.close_scissor("right")

            if joy.B:
                self.open_scissor("right")

            
    def stop_gripper(self, gripper_name, motion=0):
        cur = self.gripper_status[gripper_name].get()
        finger_pos = [cur.finger_a_status.position + motion,
                      cur.finger_b_status.position + motion,
                      cur.finger_c_status.position + motion]
                      
        self.set_gripper(gripper_name, finger_pos=finger_pos)
        
    def stop_scissor(self, gripper_name, motion=0):
        cur = self.gripper_status[gripper_name].get()
        scissor_pos = cur.scissor_status.position + motion
                      
        self.set_gripper(gripper_name, scissor_pos = scissor_pos)

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
    rospy.init_node('xbox_control')
    VJ = VictorJoystick()
    rospy.spin()


if __name__ == "__main__":
    main()


