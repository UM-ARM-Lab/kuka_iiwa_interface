#!/usr/bin/env python

from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import rospy
import numpy as np
from victor_hardware_interface_msgs.msg import Robotiq3FingerCommand
from victor_hardware_interface_msgs.srv import GripperPosition, GripperPositionResponse

left_gripper_command_pub = rospy.Publisher("victor/left_arm/gripper_command", Robotiq3FingerCommand, queue_size=10)
right_gripper_command_pub = rospy.Publisher("victor/right_arm/gripper_command", Robotiq3FingerCommand, queue_size=10)


def get_gripper_position_command(pos):
    cmd = Robotiq3FingerCommand()

    cmd.finger_a_command.position = pos
    cmd.finger_b_command.position = pos
    cmd.finger_c_command.position = pos
    cmd.scissor_command.position = 0.5

    cmd.finger_a_command.speed = 0.0
    cmd.finger_b_command.speed = 0.0
    cmd.finger_c_command.speed = 0.0
    cmd.scissor_command.speed = 0.0

    cmd.finger_a_command.force = 1.0
    cmd.finger_b_command.force = 1.0
    cmd.finger_c_command.force = 1.0
    cmd.scissor_command.force = 1.0

    return cmd

def handle_open_left_gripper(req):
    handle_set_left_gripper_opening(0.0)
    return TriggerResponse(True, "ok")

def handle_open_right_gripper(req):
    handle_set_right_gripper_opening(0.0)
    return TriggerResponse(True, "ok")

def handle_close_left_gripper(req):
    handle_set_left_gripper_opening(1.0)
    return TriggerResponse(True, "ok")

def handle_close_right_gripper(req):
    handle_set_right_gripper_opening(1.0)
    return TriggerResponse(True, "ok")

def handle_set_left_gripper_opening(opening):
    msg = get_gripper_position_command(opening.position)
    left_gripper_command_pub.publish(msg)
    return True, "ok"

def handle_set_right_gripper_opening(opening):
    msg = get_gripper_position_command(opening)
    right_gripper_command_pub.publish(msg)
    return GripperPositionResponse(True,"ok")

def open_close_gripper_server():
    rospy.init_node('open_close_gripper_server')

    srv_open_gripper = rospy.Service('left_gripper/open', Trigger, handle_open_left_gripper)
    srv_open_gripper = rospy.Service('right_gripper/open', Trigger, handle_open_right_gripper)

    srv_close_gripper = rospy.Service('left_gripper/close', Trigger, handle_close_left_gripper)
    srv_close_gripper = rospy.Service('right_gripper/close', Trigger, handle_close_right_gripper)

    srv_set_gripper_opening = rospy.Service('left_gripper/set_opening', GripperPosition, handle_set_left_gripper_opening)
    srv_set_gripper_opening = rospy.Service('right_gripper/set_opening', GripperPosition, handle_set_right_gripper_opening)

    print("open_close_gripper_server ready.")

    rospy.spin()

if __name__ == "__main__":
    open_close_gripper_server()
