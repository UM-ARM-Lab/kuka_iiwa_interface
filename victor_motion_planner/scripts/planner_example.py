#! /usr/bin/env python

# import victor_motion_planner.openrave_planner as openrave_planner
import victor_motion_planner.openrave_planner
import rospy
import numpy as np
import time
from tf.transformations import compose_matrix
from math import pi
import math
from victor_hardware_interface.msg import *

config1 = [1.60963234611, 0.533623140261, -0.742047506794, -1.16109858085, -0.323169964304, 1.09790965774, 0.219634110827]

config3 = [1.3614709264, -1.17878472014, -1.00180420345, -1.20442928365, -0.917654439591, 1.39904650482, -1.04936635634]



left_hand_start = compose_matrix(angles=[-pi/2, -pi/4, -pi],
                                 translate=[.63, .33, .72]) 

right_hand_start = compose_matrix(angles=[pi/2, pi/4, pi],
                                  translate=[.63, -.33, .72])




def run_planner():
    global config1, config2, log_one_data
    
    rospy.init_node("openrave_planner")

    planner = victor_motion_planner.openrave_planner.Planner()
    planner.open_left_gripper(blocking=False)
    planner.open_right_gripper(blocking=False)


    planner.set_manipulator("left_arm")
    planner.plan_to_configuration(config1, execute=True)
    planner.plan_to_pose(left_hand_start, execute=True)

    planner.set_manipulator("right_arm")
    planner.plan_to_configuration(config3, execute=True)
    planner.plan_to_pose(right_hand_start, execute=True)


    raw_input()
    print "You have 10 seconds"
    time.sleep(10)
    planner.close_left_gripper(blocking=False)
    planner.close_right_gripper()


    time.sleep(1)
    log_one_data = True


    for i in range(100):
        planner.move_hand_straight([0, -1, 0], .001, step_size=0.001, execute=True, waitrobot=True)
        time.sleep(1)
        log_one_data = True
        time.sleep(1)
        print i

    # planner.open_left_gripper(blocking=False)
    # planner.open_right_gripper()
    



    print "plan complete"
    # planner.start()
    # rospy.spin()
    # config1 = 


if __name__ == "__main__":
    run_planner()
