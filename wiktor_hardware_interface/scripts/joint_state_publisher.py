#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from wiktor_hardware_interface.msg import MotionStatus
from wiktor_hardware_interface.msg import Robotiq3FingerStatus
from threading import Lock
from math import pi

class WiktorJointStatePublisher:
    def __init__(self, rate):
        self.joint_names = [
                'wiktor_left_arm_joint_1',  'wiktor_left_arm_joint_2',  'wiktor_left_arm_joint_3',  'wiktor_left_arm_joint_4',  'wiktor_left_arm_joint_5',  'wiktor_left_arm_joint_6',  'wiktor_left_arm_joint_7',
                'wiktor_right_arm_joint_1', 'wiktor_right_arm_joint_2', 'wiktor_right_arm_joint_3', 'wiktor_right_arm_joint_4', 'wiktor_right_arm_joint_5', 'wiktor_right_arm_joint_6', 'wiktor_right_arm_joint_7',
                'wiktor_left_gripper_fingerA_joint_2', 'wiktor_left_gripper_fingerA_joint_3', 'wiktor_left_gripper_fingerA_joint_4',
                'wiktor_left_gripper_fingerB_knuckle', 'wiktor_left_gripper_fingerB_joint_2', 'wiktor_left_gripper_fingerB_joint_3', 'wiktor_left_gripper_fingerB_joint_4',
                'wiktor_left_gripper_fingerC_knuckle', 'wiktor_left_gripper_fingerC_joint_2', 'wiktor_left_gripper_fingerC_joint_3', 'wiktor_left_gripper_fingerC_joint_4',
                'wiktor_right_gripper_fingerA_joint_2', 'wiktor_right_gripper_fingerA_joint_3', 'wiktor_right_gripper_fingerA_joint_4',
                'wiktor_right_gripper_fingerB_knuckle', 'wiktor_right_gripper_fingerB_joint_2', 'wiktor_right_gripper_fingerB_joint_3', 'wiktor_right_gripper_fingerB_joint_4',
                'wiktor_right_gripper_fingerC_knuckle', 'wiktor_right_gripper_fingerC_joint_2', 'wiktor_right_gripper_fingerC_joint_3', 'wiktor_right_gripper_fingerC_joint_4']

        self.rate = rate
        self.joint_state_pub    = rospy.Publisher("joint_states", JointState, queue_size = 1)
        self.left_arm_sub       = rospy.Subscriber("left_arm/motion_status", MotionStatus, self.left_arm_motion_status_callback)
        self.right_arm_sub      = rospy.Subscriber("right_arm/motion_status", MotionStatus, self.right_arm_motion_status_callback)
        self.left_gripper_sub   = rospy.Subscriber("left_arm/gripper_status", Robotiq3FingerStatus, self.left_gripper_motion_status_callback)
        self.right_gripper_sub  = rospy.Subscriber("right_arm/gripper_status", Robotiq3FingerStatus, self.right_gripper_motion_status_callback)

        self.joint_state_lock = Lock()
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [0] * 36
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = []

        # Set the default values for the left arm, just in case it is not publishing data
        self.joint_state_msg.position[0] = pi / 2
        self.joint_state_msg.position[1] = -pi / 2


        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.publish_joint_values()
            rate.sleep()

    def left_arm_motion_status_callback(self, motion_status):
        self.set_arm_position_values(motion_status, offset = 0)

    def right_arm_motion_status_callback(self, motion_status):
        self.set_arm_position_values(motion_status, offset = 7)

    def left_gripper_motion_status_callback(self, gripper_status):
        self.set_gripper_position_values(gripper_status, offset = 14)

    def right_gripper_motion_status_callback(self, gripper_status):
        self.set_gripper_position_values(gripper_status, offset = 25)


    def set_arm_position_values(self, motion_status, offset):
        with self.joint_state_lock:
            self.joint_state_msg.position[offset + 0] = motion_status.measured_joint_position.joint_1
            self.joint_state_msg.position[offset + 1] = motion_status.measured_joint_position.joint_2
            self.joint_state_msg.position[offset + 2] = motion_status.measured_joint_position.joint_3
            self.joint_state_msg.position[offset + 3] = motion_status.measured_joint_position.joint_4
            self.joint_state_msg.position[offset + 4] = motion_status.measured_joint_position.joint_5
            self.joint_state_msg.position[offset + 5] = motion_status.measured_joint_position.joint_6
            self.joint_state_msg.position[offset + 6] = motion_status.measured_joint_position.joint_7

    def set_gripper_position_values(self, gripper_status, offset):
        with self.joint_state_lock:
            self.joint_state_msg.position[offset + 0] = gripper_status.finger_a_status.position*2.0
            self.joint_state_msg.position[offset + 4] = gripper_status.finger_b_status.position*2.0
            self.joint_state_msg.position[offset + 8] = gripper_status.finger_c_status.position*2.0
            # self.joint_state_msg.position[offset + 3] = gripper_status.scissor_status.position
            # self.joint_state_msg.position[offset + 7] = gripper_status.scissor_status.position


    def publish_joint_values(self):
        with self.joint_state_lock:
            self.joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_state_pub.publish(self.joint_state_msg)


if __name__ == '__main__':
    rospy.init_node('wiktor_joint_state_publisher')
    rospy.loginfo('Starting the wiktor joint state broadcaster...')
    #Get the parameters from the server
    rate = rospy.get_param("~rate", 10.0)
    WiktorJointStatePublisher(rate)
