#!/usr/bin/env python

import argparse
import sys

import rospy
from victor_fake_hardware_interface.commandline_args_utils import control_mode_arg, control_mode_strings
from victor_fake_hardware_interface.minimal_fake_arm_interface import MinimalFakeControlModeInterface as Fcmi


def main(args):
    rospy.init_node("minimal_fake_arm_interface")

    interfaces = {}

    arm_names = ["left_arm", "right_arm"]

    for arm in arm_names:
        interfaces[arm] = Fcmi(control_mode_status_topic=arm + "/control_mode_status",
                               get_control_mode_service_topic=arm + "/get_control_mode_service",
                               set_control_mode_service_topic=arm + "/set_control_mode_service",
                               initial_control_mode=control_mode_arg(args.initial_control_mode))

    for arm in arm_names:
        interfaces[arm].start_feedback_threads()

    rospy.loginfo("Publishing data...")
    rospy.spin()

    for arm in arm_names:
        interfaces[arm].join_feedback_threads()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--initial-control-mode', type=str, choices=control_mode_strings, default='JOINT_POSITION')
    main(parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:]))
