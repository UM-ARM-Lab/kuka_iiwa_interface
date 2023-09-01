#! /usr/bin/env python

"""
This change the impedance controller according to the specified stiffness level (STIFF, MEDIUM, SOFT)
"""

import colorama
from colorama import Fore

import rospy
from arm_robots.victor import Victor
from victor_hardware_interface_msgs.srv import ImpedanceSwitch, ImpedanceSwitchRequest, ImpedanceSwitchResponse
from victor_hardware_interface.victor_utils import Stiffness


def handle_impedance_switch(req):
    colorama.init(autoreset=True)
    victor = Victor(robot_namespace='victor')
    victor.connect()
    k = input("You are about to switch impedance controller. Are you sure? [y/N]")
    if k == 'Y' or k == 'y':
        rospy.loginfo(Fore.CYAN + "You chose yes...")
        if req.stiffness == "SOFT" or req.stiffness == "soft":
            stiff = Stiffness.SOFT
        elif req.stiffness == "MEDIUM" or req.stiffness == "medium":
            stiff = Stiffness.MEDIUM
        elif req.stiffness == "STIFF" or req.stiffness == "stiff":
            stiff = Stiffness.STIFF
        else:
            print("specified stiffness not found", req.stiffness)
        victor.impedance_switch(stiffness=stiff)
        rospy.loginfo("Done")
    else:
        rospy.loginfo("Answered 'no', aborting")
    return ImpedanceSwitchResponse(True, "ok")

def impedance_switch_server():
    rospy.init_node('impedance_switch_server')
    srv_switch = rospy.Service('victor/impedance_switch', ImpedanceSwitch, handle_impedance_switch)
    print("impedance_switch_server ready.")
    rospy.spin()

if __name__ == "__main__":
    impedance_switch_server()
