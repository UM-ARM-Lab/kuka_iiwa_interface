#! /usr/bin/env python

# ROS node to turn joystick msgs into Messages for Victor

import rospy
from arc_utilities import ros_helpers
from victor_hardware_interface.msg import Robotiq3FingerStatus, Robotiq3FingerCommand
from sensor_msgs.msg import Joy


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

    def joy_callback(self, joy):
        A, B, X, Y, LB, RB, back, start, power, stick_button_left, stick_button_right, \
            dleft, dright, dup, ddown = joy.buttons
        left_hor, left_vert, LT, right_hor, right_vert,  RT, d_hor, d_vert = joy.axes

        enable_finger_open_close_control = rospy.get_param("~enable_finger_open_close_control", True)
        enable_scissor_open_close_control = rospy.get_param("~enable_scissor_open_close_control", True)

        rospy.loginfo_throttle(self.output_throttle_period,
                               "Finger open close control enabled:  " + str(enable_finger_open_close_control))
        rospy.loginfo_throttle(self.output_throttle_period,
                               "Scissor open close control enabled: " + str(enable_scissor_open_close_control))

        if enable_finger_open_close_control:
            self.finger_open_close_callback(LT, LB, RT, RB)

        if enable_scissor_open_close_control:
            self.scissor_open_close_callback(X, Y, A, B)

    def finger_open_close_callback(self, LT, LB, RT, RB):
        # Open and close the gripper fingers
        if LT == -1:
            self.close_gripper("left")

        if LB:
            self.open_gripper("left")

        if not (LT == -1 or LB):
            self.stop_gripper("left")

        if RT == -1:
            self.close_gripper("right")

        if RB:
            self.open_gripper("right")

        if not (RT == -1 or RB):
            self.stop_gripper("right")

    def scissor_open_close_callback(self, X, Y, A, B):
        # Open and close the scissors on the gripper
        if X:
            self.close_scissor("left")

        if Y:
            self.open_scissor("left")

        if not (X or Y):
            self.stop_scissor("left")

        if A:
            self.close_scissor("right")

        if B:
            self.open_scissor("right")

        if not (A or B):
            self.stop_scissor("right")

    def close_gripper(self, gripper_name, **kwargs):
        self.set_gripper(gripper_name, finger_pos=(1.0, 1.0, 1.0), **kwargs)

    def open_gripper(self, gripper_name, **kwargs):
        self.set_gripper(gripper_name, finger_pos=(0.0, 0.0, 0.0), **kwargs)

    def stop_gripper(self, gripper_name):
        status_msg = self.gripper_status[gripper_name].get()
        a = status_msg.finger_a_status.position
        b = status_msg.finger_b_status.position
        c = status_msg.finger_c_status.position
        self.set_gripper(gripper_name, finger_pos=(a, b,c ))

    def close_scissor(self, gripper_name, **kwargs):
        self.set_gripper(gripper_name, scissor_pos=1.0, **kwargs)

    def open_scissor(self, gripper_name, **kwargs):
        self.set_gripper(gripper_name, scissor_pos=0.0, **kwargs)

    def stop_scissor(self, gripper_name):
        status_msg = self.gripper_status[gripper_name].get()
        s = status_msg.scissor_status.position
        self.set_gripper(gripper_name, scissor_pos=s)

    def set_gripper(self, gripper_name, finger_pos=None, scissor_pos=None, blocking=True, continuous=False):
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
        self.gripper_status[gripper_name].get()
        while not self.gripper_status[gripper_name].has_new_data():
            rospy.sleep(0.01)

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


