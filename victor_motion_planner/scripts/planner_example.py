#! /usr/bin/env python

import openrave_planner
import rospy

config1 = [1.3565332459566835, -0.1543670633363234, 2.071383692805881, 0.6187004711779157,
             -2.8008742367028634, 0.2583977454503356, 0.0]
config2 = [1.3565332459566835, -0.1543670633363234, 2.071383692805881, 0.6187004711779157,
             -2.8008742367028634, 0.2583977454503356, .2]

config3 = [1.3565332459566835, -0.1543670633363234, 2.071383692805881, 0.6187004711779157,
             -2.8008742367028634, 0.4583977454503356, 1]
config4 = [1.3565332459566835, -0.1543670633363234, 2.071383692805881, 0.6187004711779157,
             -2.8008742367028634, 0.3583977454503356, 1]

def runPlanner():
    global config1, config2
    
    rospy.init_node("openrave_planner")
    
    planner = openrave_planner.Planner()

    rospy.loginfo("moving left arm")
    planner.plan_to_configuration(config1, execute=True, use_fake_obstacle=False)
    planner.plan_to_configuration(config2, execute=True, use_fake_obstacle=False)

    planner.set_manipulator("right_arm")

    rospy.loginfo("moving right arm")
    planner.plan_to_configuration(config3, execute=True, use_fake_obstacle=False)
    planner.plan_to_configuration(config4, execute=True, use_fake_obstacle=False)



    print "plan complete"
    # planner.start()
    # rospy.spin()
    # config1 = 


if __name__ == "__main__":
    runPlanner()
