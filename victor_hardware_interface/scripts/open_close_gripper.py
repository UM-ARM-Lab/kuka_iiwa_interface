#!/usr/bin/env python

from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import rospy
import numpy as np
#from arm_robots.victor import Victor
from arc_utilities.listener import Listener

from victor_hardware_interface_msgs.msg import Robotiq3FingerCommand, Robotiq3FingerStatus

from victor_hardware_interface_msgs.srv import GripperPosition, GripperPositionResponse

global left_gripper_command_pub
global right_gripper_command_pub
global left_gripper_status_listener
global right_gripper_status_listener


#global victor

def get_gripper_position_command(fingers_pos, scissor=0.5):
    cmd = Robotiq3FingerCommand()

    cmd.finger_a_command.position = fingers_pos
    cmd.finger_b_command.position = fingers_pos
    cmd.finger_c_command.position = fingers_pos
    cmd.scissor_command.position = scissor

    cmd.finger_a_command.speed = 0.0
    cmd.finger_b_command.speed = 0.0
    cmd.finger_c_command.speed = 0.0
    cmd.scissor_command.speed = 0.0

    cmd.finger_a_command.force = 1.0
    cmd.finger_b_command.force = 1.0
    cmd.finger_c_command.force = 1.0
    cmd.scissor_command.force = 1.0

    return cmd

def get_gripper_position_error(msg):
    finger_a_error = abs(msg.finger_a_status.position_request - msg.finger_a_status.position)
    finger_b_error = abs(msg.finger_b_status.position_request - msg.finger_b_status.position)
    finger_c_error = abs(msg.finger_c_status.position_request - msg.finger_c_status.position)
    scissor_error = abs(msg.scissor_status.position_request - msg.scissor_status.position)
    return max( finger_a_error, finger_b_error, finger_c_error, scissor_error )

def handle_open_left_gripper(req):
    res = handle_set_left_gripper_opening(0.0)
    return TriggerResponse(res.success, res.message)

def handle_open_right_gripper(req):
    res = handle_set_right_gripper_opening(0.0)
    return TriggerResponse(res.success, res.message)

def handle_close_left_gripper(req):
    #r = rospy.Rate(10) # 10hz
    #t0 = rospy.get_rostime()
    #rospy.loginfo("closing left gripper")
    #if victor.is_left_gripper_closed():
    #    rospy.loginfo("closing left gripper")
    #global victor
    #while (not victor.is_left_gripper_closed()) and (rospy.get_rostime()-t0).to_sec()<10.0:
    #    victor.close_left_gripper()
    #    r.sleep()
    #if not victor.is_left_gripper_closed():
    #    return  TriggerResponse(False, "not closed")
    #return TriggerResponse(True, "ok")

    res = handle_set_left_gripper_opening(1.0)
    return TriggerResponse(res.success, res.message)

def handle_close_right_gripper(req):
    res = handle_set_right_gripper_opening(1.0)
    return TriggerResponse(res.success, res.message)

def handle_set_left_gripper_opening(opening):
    msg = get_gripper_position_command(opening)
    global left_gripper_command_pub
    left_gripper_command_pub.publish(msg)
    r = rospy.Rate(10) # 10hz
    t0 = rospy.get_rostime()
    global left_gripper_status_listener
    while (rospy.get_rostime()-t0).to_sec()<20.0:
        if get_gripper_position_error(left_gripper_status_listener.get()) < 0.05:
                return GripperPositionResponse(True,"ok")
        r.sleep()
    print("not reached reference opening")
    return  GripperPositionResponse(False, "not reached reference opening")

def handle_set_right_gripper_opening(opening):
    msg = get_gripper_position_command(opening)
    global right_gripper_command_pub
    right_gripper_command_pub.publish(msg)
    r = rospy.Rate(10) # 10hz
    t0 = rospy.get_rostime()
    global right_gripper_status_listener
    while (rospy.get_rostime()-t0).to_sec()<20.0:
        if get_gripper_position_error(right_gripper_status_listener.get()) < 0.05:
                return GripperPositionResponse(True,"ok")
        r.sleep()
    print("not reached reference opening")
    return  GripperPositionResponse(False, "not reached reference opening")

def open_close_gripper_server():
    rospy.init_node('open_close_gripper_server')

    srv_open_gripper = rospy.Service('left_gripper/open', Trigger, handle_open_left_gripper)
    srv_open_gripper = rospy.Service('right_gripper/open', Trigger, handle_open_right_gripper)

    srv_close_gripper = rospy.Service('left_gripper/close', Trigger, handle_close_left_gripper)
    srv_close_gripper = rospy.Service('right_gripper/close', Trigger, handle_close_right_gripper)

    srv_set_gripper_opening = rospy.Service('left_gripper/set_opening', GripperPosition, handle_set_left_gripper_opening)
    srv_set_gripper_opening = rospy.Service('right_gripper/set_opening', GripperPosition, handle_set_right_gripper_opening)

    global left_gripper_command_pub
    global right_gripper_command_pub
    global left_gripper_status_listener
    global right_gripper_status_listener

    left_gripper_command_pub = rospy.Publisher("victor/left_arm/gripper_command", Robotiq3FingerCommand, queue_size=10)
    right_gripper_command_pub = rospy.Publisher("victor/right_arm/gripper_command", Robotiq3FingerCommand, queue_size=10)
    left_gripper_status_listener = Listener("/victor/left_arm/gripper_status", Robotiq3FingerStatus)
    right_gripper_status_listener = Listener("/victor/right_arm/gripper_status", Robotiq3FingerStatus)



    #global victor
    #victor = Victor(robot_namespace='victor')
    #victor.connect()

    print("open_close_gripper_server ready.")

    rospy.spin()

if __name__ == "__main__":
    open_close_gripper_server()
