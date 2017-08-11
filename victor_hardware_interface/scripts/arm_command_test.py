#!/usr/bin/env python

#####################################################
#                                                   #
#   Copyright (c) 2017, UM-ARM-LAB                  #
#                                                   #
#   Regression test for arm control modes           #
#                                                   #
#####################################################

# ros
import rospy

# victor
import victor_hardware_interface.msg
import victor_hardware_interface.srv
import victor_openrave

# openrave
import openravepy as rave

# standard libraries
import random
import math
import numpy as np
import IPython


class ControlModeTester(object):

    def __init__(self):
        print("Setting up control mode & motion command...")
        rospy.init_node("control_mode_tester")
        self.victor_sim = victor_openrave.victor_openrave()

        self.set_control_mode_server = rospy.ServiceProxy("set_control_mode_service",
                                                          victor_hardware_interface.srv.SetControlMode)
        self.set_control_mode_server.wait_for_service()
        self.get_control_mode_server = rospy.ServiceProxy("get_control_mode_service",
                                                          victor_hardware_interface.srv.GetControlMode)
        self.get_control_mode_server.wait_for_service()
        self.motion_command_pub = rospy.Publisher("right_arm/motion_command",
                                                  victor_hardware_interface.msg.MotionCommand,
                                                  queue_size=1)

        self.arm_command = victor_hardware_interface.msg.MotionCommand()

        

        print("...Finished setting up services & publishers")

    def send_random_joint_position(self):
        
        if(self.arm_command.control_mode != victor_hardware_interface.msg.MotionCommand.JOINT_POSITION or 
           self.arm_command.control_mode != victor_hardware_interface.msg.MotionCommand.JOINT_IMPEDANCE):
        
            arm_joint_limit_margin = 5
            joint_limits = [(-170+arm_joint_limit_margin,170-arm_joint_limit_margin),
                            (-120+arm_joint_limit_margin,120-arm_joint_limit_margin),
                            (-170+arm_joint_limit_margin,170-arm_joint_limit_margin),
                            (-120+arm_joint_limit_margin,120-arm_joint_limit_margin),
                            (-170+arm_joint_limit_margin,170-arm_joint_limit_margin),
                            (-120+arm_joint_limit_margin,120-arm_joint_limit_margin),
                            (-175+arm_joint_limit_margin,175-arm_joint_limit_margin)]

            joint_values = [self.arm_command.joint_position.joint_1,
                           self.arm_command.joint_position.joint_2,
                           self.arm_command.joint_position.joint_3,
                           self.arm_command.joint_position.joint_4,
                           self.arm_command.joint_position.joint_5,
                           self.arm_command.joint_position.joint_6,
                           self.arm_command.joint_position.joint_7]

            joint_num = len(joint_limits)

            while(True):
                for i in range(joint_num):
                    lower_limit = joint_limits[i][0]
                    upper_limit = joint_limits[i][1]

                    joint_values[i] = (random.random() * (upper_limit - lower_limit) + lower_limit) * math.pi/180.0

                victor_joint_values = self.victor_sim.robot.GetDOFValues()

                for i,index in enumerate(self.victor_sim.victor_right_arm_joint_indices):
                    victor_joint_values[index] = joint_values[i]

                self.victor_sim.robot.SetDOFValues(victor_joint_values)

                if(not self.victor_sim.env.CheckCollision(self.victor_sim.robot) and not self.victor_sim.robot.CheckSelfCollision()):
                    print("Send Joint Command: (%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f)"%(joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5],joint_values[6]))
                    self.motion_command_pub.publish(self.arm_command)
                    break
        else:
            print("The joint position command should be sent only in JOINT_POSITION and JOINT_IMPEDANCE control mode. Ignore the command...")

    def send_random_cartesian_pose(self):
        
        if(self.arm_command.control_mode != victor_hardware_interface.msg.MotionCommand.CARTESIAN_POSE or 
           self.arm_command.control_mode != victor_hardware_interface.msg.MotionCommand.CARTESIAN_IMPEDANCE):
        
            arm_joint_limit_margin = 5
            joint_limits = [(-170+arm_joint_limit_margin,170-arm_joint_limit_margin),
                            (-120+arm_joint_limit_margin,120-arm_joint_limit_margin),
                            (-170+arm_joint_limit_margin,170-arm_joint_limit_margin),
                            (-120+arm_joint_limit_margin,120-arm_joint_limit_margin),
                            (-170+arm_joint_limit_margin,170-arm_joint_limit_margin),
                            (-120+arm_joint_limit_margin,120-arm_joint_limit_margin),
                            (-175+arm_joint_limit_margin,175-arm_joint_limit_margin)]

            joint_values = [0,0,0,0,0,0,0]

            joint_num = len(joint_limits)

            while(True):
                for i in range(joint_num):
                    lower_limit = joint_limits[i][0]
                    upper_limit = joint_limits[i][1]

                    joint_values[i] = (random.random() * (upper_limit - lower_limit) + lower_limit) * math.pi/180.0

                victor_joint_values = self.victor_sim.robot.GetDOFValues()

                for i,index in enumerate(self.victor_sim.victor_right_arm_joint_indices):
                    victor_joint_values[index] = joint_values[i]

                self.victor_sim.robot.SetDOFValues(victor_joint_values)

                if(not self.victor_sim.env.CheckCollision(self.victor_sim.robot) and not self.victor_sim.robot.CheckSelfCollision()):
                    # Call FK to get the pose
                    ee_transform = self.victor_sim.robot.GetManipulator('right_arm_palm_surface_kuka').GetTransform()
                    self.arm_command.cartesian_pose.x = ee_transform[0,3]
                    self.arm_command.cartesian_pose.y = ee_transform[1,3]
                    self.arm_command.cartesian_pose.z = ee_transform[2,3]

                    [a,b,c] = rave.axisAngleFromRotationMatrix(ee_transform[0:3,0:3])

                    self.arm_command.cartesian_pose.a = a
                    self.arm_command.cartesian_pose.b = b
                    self.arm_command.cartesian_pose.c = c

                    print("Send Joint Command: (%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f)"%(self.arm_command.cartesian_pose.x,self.arm_command.cartesian_pose.y,self.arm_command.cartesian_pose.z,a,b,c))
                    self.motion_command_pub.publish(self.arm_command)
                    break

        else:
            print("The cartesian pose command should be sent only in CARTESIAN_POSE and CARTESIAN_IMPEDANCE control mode. Ignore the command...")

        

    def joint_position_test(self):
        raw_input("Run joint position test - Press ENTER to continue...")
        control_mode_command = victor_hardware_interface.msg.ControlModeCommand()
        control_mode_command.control_mode = victor_hardware_interface.msg.ControlModeCommand.JOINT_POSITION
        control_mode_command.joint_path_execution_params.joint_relative_velocity = 0.5
        control_mode_command.joint_path_execution_params.joint_relative_acceleration = 0.5
        control_mode_command.joint_path_execution_params.override_joint_acceleration = 0.0
        request = victor_hardware_interface.srv.SetControlModeRequest()
        request.new_control_mode = control_mode_command
        response = self.set_control_mode_server.call(request)
        print("SetControlMode response: " + str(response))
        assert(response.success is True)
        self.arm_command.control_mode = victor_hardware_interface.msg.MotionCommand.JOINT_POSITION
        print("...Joint position test complete")

    def joint_impedance_test(self):
        raw_input("Run joint impedance test - Press ENTER to continue...")
        control_mode_command = victor_hardware_interface.msg.ControlModeCommand()
        control_mode_command.control_mode = victor_hardware_interface.msg.ControlModeCommand.JOINT_IMPEDANCE
        control_mode_command.joint_path_execution_params.joint_relative_velocity = 0.5
        control_mode_command.joint_path_execution_params.joint_relative_acceleration = 0.5
        control_mode_command.joint_path_execution_params.override_joint_acceleration = 0.0
        control_mode_command.joint_impedance_params.joint_damping.joint_1 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_2 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_3 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_4 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_5 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_6 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_7 = 0.7
        control_mode_command.joint_impedance_params.joint_stiffness.joint_1 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_2 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_3 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_4 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_5 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_6 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_7 = 1.0
        request = victor_hardware_interface.srv.SetControlModeRequest()
        request.new_control_mode = control_mode_command
        response = self.set_control_mode_server.call(request)
        print("SetControlMode response: " + str(response))
        assert (response.success is True)
        self.arm_command.control_mode = victor_hardware_interface.msg.MotionCommand.JOINT_IMPEDANCE
        print("...Joint impedance test complete")

    def cartesian_pose_test(self):
        raw_input("Run cartesian pose test - Press ENTER to continue...")
        control_mode_command = victor_hardware_interface.msg.ControlModeCommand()
        control_mode_command.control_mode = victor_hardware_interface.msg.ControlModeCommand.CARTESIAN_POSE
        control_mode_command.cartesian_path_execution_params.max_velocity.x = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.y = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.z = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.a = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.b = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.c = 0.5
        control_mode_command.cartesian_path_execution_params.max_nullspace_velocity = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.x = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.y = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.z = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.a = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.b = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.c = 0.5
        control_mode_command.cartesian_path_execution_params.max_nullspace_acceleration = 0.5
        request = victor_hardware_interface.srv.SetControlModeRequest()
        request.new_control_mode = control_mode_command
        response = self.set_control_mode_server.call(request)
        print("SetControlMode response: " + str(response))
        assert (response.success is True)
        self.arm_command.control_mode = victor_hardware_interface.msg.MotionCommand.CARTESIAN_POSE
        print("...Cartesian pose test complete")

    def cartesian_impedance_test(self):
        raw_input("Run cartesian impedance test - Press ENTER to continue...")
        control_mode_command = victor_hardware_interface.msg.ControlModeCommand()
        control_mode_command.control_mode = victor_hardware_interface.msg.ControlModeCommand.CARTESIAN_IMPEDANCE
        control_mode_command.cartesian_path_execution_params.max_velocity.x = 100.0
        control_mode_command.cartesian_path_execution_params.max_velocity.y = 100.0
        control_mode_command.cartesian_path_execution_params.max_velocity.z = 100.0
        control_mode_command.cartesian_path_execution_params.max_velocity.a = 100.0
        control_mode_command.cartesian_path_execution_params.max_velocity.b = 100.0
        control_mode_command.cartesian_path_execution_params.max_velocity.c = 100.0
        control_mode_command.cartesian_path_execution_params.max_nullspace_velocity = 250.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.x = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.y = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.z = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.a = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.b = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.c = 10.0
        control_mode_command.cartesian_path_execution_params.max_nullspace_acceleration = 10.0
        control_mode_command.cartesian_impedance_params.cartesian_damping.x = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.y = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.z = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.a = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.b = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.c = 0.7
        control_mode_command.cartesian_impedance_params.nullspace_damping = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.x = 1.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.y = 1.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.z = 1.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.a = 1.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.b = 1.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.c = 1.0
        control_mode_command.cartesian_impedance_params.nullspace_stiffness = 1.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.x = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.y = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.z = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.a = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.b = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.c = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.x = 100.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.y = 100.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.z = 100.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.a = 100.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.b = 100.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.c = 100.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.x = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.y = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.z = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.a = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.b = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.c = 10.0
        control_mode_command.cartesian_control_mode_limits.stop_on_max_control_force = False
        request = victor_hardware_interface.srv.SetControlModeRequest()
        request.new_control_mode = control_mode_command
        response = self.set_control_mode_server.call(request)
        print("SetControlMode response: " + str(response))
        assert (response.success is True)
        self.arm_command.control_mode = victor_hardware_interface.msg.MotionCommand.CARTESIAN_IMPEDANCE
        print("...Cartesian impedance test complete")

    def run_all_tests(self):
        print("Starting arm control regression tests...")
        raw_input("Is the arm safely away from obstacles? Press ENTER to continue...")

        test_run = 10

        self.joint_position_test()
        for i in range(test_run):
            self.send_random_joint_position()

        self.joint_impedance_test()
        for i in range(test_run):
            self.send_random_joint_position()

        self.cartesian_pose_test()
        for i in range(test_run):
            self.send_random_cartesian_pose()

        self.cartesian_impedance_test()
        for i in range(test_run):
            self.send_random_cartesian_pose()

        print("...Finished arm control regression tests")


if __name__ == '__main__':
    tester = ControlModeTester()
    tester.run_all_tests()
