#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from victor_hardware_interface.msg import MotionStatus
from victor_hardware_interface.msg import Robotiq3FingerStatus
from threading import Lock
from math import pi

def computeFingerAngles(control):
    # the control is from 0 to 1, however, the relationship takes 0 to 255
    g = control * 255
    max_angle = [70.0, 90.0, 43.0]
    min_3 = -55.0
    m1 = max_angle[0] / 140.0
    m2 = max_angle[1] / 100.0
    if g <= 110:
        theta1 = m1 * g
        theta2 = 0
        theta3 = -m1 * g
    elif 110 < g <= 140:
        theta1 = m1 * g
        theta2 = 0
        theta3 = min_3
    elif 140 < g <= 240:
        theta1 = max_angle[0]
        theta2 = m2 * (g-140)
        theta3 = min_3
    else:
        theta1 = max_angle[0]
        theta2 = max_angle[1]
        theta3 = min_3
    return [theta1 * pi / 180.0, theta2 * pi / 180.0, theta3 * pi / 180.0]

def computeScissorAngle(control):
    # 0 for fully open with -16, 1 for fully close with 10
    return (26.0 * control - 16.0) * pi / 180.0


class VictorJointStatePublisher:
    def __init__(self, rate):
        self.joint_names = [
                'victor_left_arm_joint_1',  'victor_left_arm_joint_2',  'victor_left_arm_joint_3',  'victor_left_arm_joint_4',  'victor_left_arm_joint_5',  'victor_left_arm_joint_6',  'victor_left_arm_joint_7',
                'victor_right_arm_joint_1', 'victor_right_arm_joint_2', 'victor_right_arm_joint_3', 'victor_right_arm_joint_4', 'victor_right_arm_joint_5', 'victor_right_arm_joint_6', 'victor_right_arm_joint_7',
                                                        'victor_left_gripper_fingerA_joint_2',  'victor_left_gripper_fingerA_joint_3',  'victor_left_gripper_fingerA_joint_4',
                'victor_left_gripper_fingerB_knuckle',  'victor_left_gripper_fingerB_joint_2',  'victor_left_gripper_fingerB_joint_3',  'victor_left_gripper_fingerB_joint_4',
                'victor_left_gripper_fingerC_knuckle',  'victor_left_gripper_fingerC_joint_2',  'victor_left_gripper_fingerC_joint_3',  'victor_left_gripper_fingerC_joint_4',
                                                        'victor_right_gripper_fingerA_joint_2', 'victor_right_gripper_fingerA_joint_3', 'victor_right_gripper_fingerA_joint_4',
                'victor_right_gripper_fingerB_knuckle', 'victor_right_gripper_fingerB_joint_2', 'victor_right_gripper_fingerB_joint_3', 'victor_right_gripper_fingerB_joint_4',
                'victor_right_gripper_fingerC_knuckle', 'victor_right_gripper_fingerC_joint_2', 'victor_right_gripper_fingerC_joint_3', 'victor_right_gripper_fingerC_joint_4']

        self.joint_state_lock = Lock()
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [0] * len(self.joint_names)
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = [0] * len(self.joint_names)

        self.rate = rate
        self.joint_state_pub    = rospy.Publisher("joint_states", JointState, queue_size = 1)
        self.left_arm_sub       = rospy.Subscriber("left_arm/motion_status", MotionStatus, self.left_arm_motion_status_callback)
        self.right_arm_sub      = rospy.Subscriber("right_arm/motion_status", MotionStatus, self.right_arm_motion_status_callback)
        self.left_gripper_sub   = rospy.Subscriber("left_arm/gripper_status", Robotiq3FingerStatus, self.left_gripper_motion_status_callback)
        self.right_gripper_sub  = rospy.Subscriber("right_arm/gripper_status", Robotiq3FingerStatus, self.right_gripper_motion_status_callback)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.publish_joint_values()
            rate.sleep()

    def left_arm_motion_status_callback(self, motion_status):
        self.set_arm_position_values(motion_status, offset = 0)
        self.set_arm_effort_values(motion_status, offset = 0)

    def right_arm_motion_status_callback(self, motion_status):
        self.set_arm_position_values(motion_status, offset = 7)
        self.set_arm_effort_values(motion_status, offset = 7)

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
            joint_a = computeFingerAngles(gripper_status.finger_a_status.position)
            self.joint_state_msg.position[offset + 0] = joint_a[0]
            self.joint_state_msg.position[offset + 1] = joint_a[1]
            self.joint_state_msg.position[offset + 2] = joint_a[2]

            self.joint_state_msg.position[offset + 3] = computeScissorAngle(gripper_status.scissor_status.position)
            joint_b = computeFingerAngles(gripper_status.finger_b_status.position)
            self.joint_state_msg.position[offset + 4] = joint_b[0]
            self.joint_state_msg.position[offset + 5] = joint_b[1]
            self.joint_state_msg.position[offset + 6] = joint_b[2]

            self.joint_state_msg.position[offset + 7] = computeScissorAngle(gripper_status.scissor_status.position)
            joint_c = computeFingerAngles(gripper_status.finger_c_status.position)
            self.joint_state_msg.position[offset + 8] = joint_c[0]
            self.joint_state_msg.position[offset + 9] = joint_c[1]
            self.joint_state_msg.position[offset + 10] = joint_c[2]


            # self.joint_state_msg.position[offset + 3] = gripper_status.scissor_status.position
            # self.joint_state_msg.position[offset + 7] = gripper_status.scissor_status.position
    def set_arm_effort_values(self, motion_status, offset):
        ## We use measured joint torque for now. As Kuka mentioned, it is the currently measured "raw" torque sensor data. There is an estimated_external_torque messange, which is the current external torque sensor data for this robot.
        with self.joint_state_lock:
            self.joint_state_msg.effort[offset + 0] = motion_status.measured_joint_torque.joint_1
            self.joint_state_msg.effort[offset + 1] = motion_status.measured_joint_torque.joint_2
            self.joint_state_msg.effort[offset + 2] = motion_status.measured_joint_torque.joint_3
            self.joint_state_msg.effort[offset + 3] = motion_status.measured_joint_torque.joint_4
            self.joint_state_msg.effort[offset + 4] = motion_status.measured_joint_torque.joint_5
            self.joint_state_msg.effort[offset + 5] = motion_status.measured_joint_torque.joint_6

    def publish_joint_values(self):
        with self.joint_state_lock:
            self.joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_state_pub.publish(self.joint_state_msg)


if __name__ == '__main__':
    rospy.init_node('victor_joint_state_publisher')
    rospy.loginfo('Starting the victor joint state broadcaster...')
    #Get the parameters from the server
    rate = rospy.get_param("~rate", 10.0)
    pub = VictorJointStatePublisher(rate)
    pub.run()
