#!/usr/bin/env python
import rospy
from victor_fake_hardware_interface.minimal_fake_arm_interface import MinimalFakeGripperInterface


def main():
    rospy.init_node("minimal_fake_arm_interface")

    interfaces = {}

    arm_names = ["left_arm", "right_arm"]

    for arm in arm_names:
        interfaces[arm + "/gripper"] = MinimalFakeGripperInterface(gripper_command_topic=arm + "/gripper_command",
                                                                   gripper_status_topic=arm + "/gripper_status")

    for arm in arm_names:
        interfaces[arm + "/gripper"].start_feedback_threads()

    rospy.loginfo("Publishing data...")
    rospy.spin()

    for arm in arm_names:
        interfaces[arm + "/gripper"].join_feedback_threads()


if __name__ == '__main__':
    main()
