#!/usr/bin/env python

import math
from threading import Lock
from threading import Thread

import rospy
from victor_hardware_interface_msgs.msg import *
from victor_hardware_interface_msgs.srv import *


def default_robotiq_3f_gripper_status():
    """
    TODO: This does not exactly match the output of the arm when it is first turned on. But it's close enough for now.
    :return type: Robotiq3FingerStatus
    """
    msg = Robotiq3FingerStatus()

    msg.finger_a_status.position_request = 0.0
    msg.finger_a_status.position = 0.0
    msg.finger_a_status.current = 0.0

    msg.finger_b_status = msg.finger_a_status
    msg.finger_c_status = msg.finger_a_status

    msg.scissor_status.position_request = 0.5
    msg.scissor_status.position = 0.5
    msg.scissor_status.current = 0.0

    msg.finger_a_object_status.status = Robotiq3FingerObjectStatus.AT_REQUESTED
    msg.finger_b_object_status.status = Robotiq3FingerObjectStatus.AT_REQUESTED
    msg.finger_c_object_status.status = Robotiq3FingerObjectStatus.AT_REQUESTED
    msg.scissor_object_status.status = Robotiq3FingerObjectStatus.AT_REQUESTED

    return msg


class MinimalFakeGripperInterface:
    gripper_feedback_rate = 10.0  # Hz

    def __init__(self,
                 gripper_command_topic,
                 gripper_status_topic):
        self.input_mtx = Lock()

        self.gripper_status_msg = default_robotiq_3f_gripper_status()

        self.gripper_command_sub = rospy.Subscriber(gripper_command_topic, Robotiq3FingerCommand,
                                                    self.gripper_command_callback)

        self.gripper_status_pub = rospy.Publisher(gripper_status_topic, Robotiq3FingerStatus, queue_size=1)

        self.gripper_status_thread = Thread(target=self.gripper_status_feedback_thread)

    def gripper_command_callback(self, cmd):
        """
        :type cmd: Robotiq3FingerCommand
        :return: None
        """
        with self.input_mtx:
            self.gripper_status_msg.finger_a_status.position_request = cmd.finger_a_command.position
            self.gripper_status_msg.finger_a_status.position = cmd.finger_a_command.position
            self.gripper_status_msg.finger_b_status.position_request = cmd.finger_b_command.position
            self.gripper_status_msg.finger_b_status.position = cmd.finger_b_command.position
            self.gripper_status_msg.finger_c_status.position_request = cmd.finger_c_command.position
            self.gripper_status_msg.finger_c_status.position = cmd.finger_c_command.position
            self.gripper_status_msg.scissor_status.position_request = cmd.scissor_command.position
            self.gripper_status_msg.scissor_status.position = cmd.scissor_command.position

    # TODO: populate seq
    # TODO: populate timestamp in submessages
    def gripper_status_feedback_thread(self):
        r = rospy.Rate(self.gripper_feedback_rate)
        while not rospy.is_shutdown():
            with self.input_mtx:
                self.gripper_status_msg.header.stamp = rospy.Time.now()
                self.gripper_status_pub.publish(self.gripper_status_msg)
            r.sleep()

    def start_feedback_threads(self):
        self.gripper_status_thread.start()

    def join_feedback_threads(self):
        self.gripper_status_thread.join()


class MinimalFakeArmInterface:
    cartesian_field_names = ["x", "y", "z", "a", "b", "c"]
    joint_names = ['joint_' + str(i) for i in range(1, 8)]
    control_mode_feedback_rate = 1.0  # Hz
    arm_status_feedback_rate = 100.0  # Hz

    def __init__(self,
                 arm_name,
                 control_mode_status_topic,
                 get_control_mode_service_topic,
                 set_control_mode_service_topic,
                 motion_command_topic,
                 motion_status_topic,
                 initial_control_mode=ControlMode.JOINT_POSITION,
                 ):

        self.input_mtx = Lock()

        self.control_mode_parameters_status_msg = self.default_control_mode_parameters_status(initial_control_mode)
        self.motion_status_msg = self.default_motion_status(arm_name)

        self.get_control_mode_server = rospy.Service(get_control_mode_service_topic, GetControlMode,
                                                     self.get_control_mode_service_callback)
        self.set_control_mode_server = rospy.Service(set_control_mode_service_topic, SetControlMode,
                                                     self.set_control_mode_service_callback)

        self.motion_command_sub = rospy.Subscriber(motion_command_topic, MotionCommand,
                                                   self.arm_motion_command_callback)

        self.control_status_pub = rospy.Publisher(control_mode_status_topic, ControlModeParameters, queue_size=1)
        self.motion_status_pub = rospy.Publisher(motion_status_topic, MotionStatus, queue_size=1)

        self.control_mode_thread = Thread(target=self.control_mode_feedback_thread)
        self.arm_status_thread = Thread(target=self.arm_status_feedback_thread)

    def arm_motion_command_callback(self, cmd):
        """
        :type cmd: MotionCommand
        :return: None
        """

        with self.input_mtx:
            if cmd.control_mode == self.control_mode_parameters_status_msg.control_mode:
                if cmd.control_mode.mode == ControlMode.JOINT_POSITION or cmd.control_mode.mode == ControlMode.JOINT_IMPEDANCE:
                    self.motion_status_msg.commanded_joint_position = cmd.joint_position
                    self.motion_status_msg.measured_joint_position = cmd.joint_position
                else:
                    rospy.logerr("Cartesian control modes not implemented")
            else:
                rospy.logerr("Motion command control mode " + str(cmd.control_mode) +
                             " does not match active control mode" +
                             str(self.control_mode_parameters_status_msg.control_mode))

    def get_control_mode_service_callback(self, req):
        """
        :type req: GetControlModeRequest
        :return:
        """
        with self.input_mtx:
            return GetControlModeResponse(active_control_mode=self.control_mode_parameters_status_msg,
                                          has_active_control_mode=True)

    def set_control_mode_service_callback(self, req):
        """
        :param req: SetControlModeRequest
        :return:
        """
        with self.input_mtx:
            self.control_mode_parameters_status_msg = req.new_control_mode
            self.motion_status_msg.active_control_mode = req.new_control_mode.control_mode
            return SetControlModeResponse(success=True, message="")

    # TODO: populate seq
    # TODO: populate timestamp in submessages
    def control_mode_feedback_thread(self):
        r = rospy.Rate(self.control_mode_feedback_rate)
        while not rospy.is_shutdown():
            with self.input_mtx:
                self.control_mode_parameters_status_msg.header.stamp = rospy.Time.now()
                self.control_status_pub.publish(self.control_mode_parameters_status_msg)
            r.sleep()

    # TODO: populate seq
    # TODO: populate timestamp in submessages
    def arm_status_feedback_thread(self):
        r = rospy.Rate(self.arm_status_feedback_rate)
        while not rospy.is_shutdown():
            with self.input_mtx:
                self.motion_status_msg.header.stamp = rospy.Time.now()
                self.motion_status_pub.publish(self.motion_status_msg)
            r.sleep()

    def start_feedback_threads(self):
        self.control_mode_thread.start()
        self.arm_status_thread.start()

    def join_feedback_threads(self):
        self.control_mode_thread.join()
        self.arm_status_thread.join()

    @staticmethod
    def default_control_mode_parameters_status(control_mode=ControlMode.JOINT_POSITION):
        """
        :return type: ControlModeParameters
        """
        msg = ControlModeParameters()

        msg.control_mode.mode = control_mode

        # Joint impedance parameters
        for joint in MinimalFakeArmInterface.joint_names:
            setattr(msg.joint_impedance_params.joint_stiffness, joint, 0.0)
            setattr(msg.joint_impedance_params.joint_damping, joint, 0.0)

        # Cartesian impedance parameters
        for field in MinimalFakeArmInterface.cartesian_field_names:
            setattr(msg.cartesian_impedance_params.cartesian_stiffness, field, 0.1)
            setattr(msg.cartesian_impedance_params.cartesian_damping, field, 0.1)
        msg.cartesian_impedance_params.nullspace_stiffness = 0.0
        msg.cartesian_impedance_params.nullspace_damping = 0.3

        # Cartesian control mode limits
        for field in MinimalFakeArmInterface.cartesian_field_names:
            setattr(msg.cartesian_control_mode_limits.max_path_deviation, field, 0.1)
            setattr(msg.cartesian_control_mode_limits.max_cartesian_velocity, field, 0.1)
            setattr(msg.cartesian_control_mode_limits.max_control_force, field, 0.1)
        msg.cartesian_control_mode_limits.stop_on_max_control_force = False

        # Joint path execution params
        msg.joint_path_execution_params.joint_relative_velocity = 0.1
        msg.joint_path_execution_params.joint_relative_acceleration = 0.1
        msg.joint_path_execution_params.override_joint_acceleration = 0.0

        # Cartesian path execution params
        for field in MinimalFakeArmInterface.cartesian_field_names:
            setattr(msg.cartesian_path_execution_params.max_velocity, field, 0.1)
            setattr(msg.cartesian_path_execution_params.max_acceleration, field, 0.1)
        msg.cartesian_path_execution_params.max_nullspace_velocity = 0.1
        msg.cartesian_path_execution_params.max_nullspace_acceleration = 0.1

        return msg

    @staticmethod
    def default_motion_status(arm):
        """
        The default motion status has the arm raised in the air.
        This ensures no collision in the default OpenRAVE environment
        :param arm: Which arm is this for? Determines the value of joint 2. Value inputs are "right_arm" and "left_arm"
        :return type: MotionStatus
        """
        msg = MotionStatus()

        for joint in MinimalFakeArmInterface.joint_names:
            setattr(msg.measured_joint_position, joint, 0.0)
        msg.measured_joint_position.joint_1 = -math.pi / 2.0

        # Set the position of joint 2 depending on which direction raises it into the air
        if arm == "right_arm":
            msg.measured_joint_position.joint_2 = -math.pi / 2.0
        elif arm == "left_arm":
            msg.measured_joint_position.joint_2 = math.pi / 2.0
        else:
            raise ValueError("arm must be either right_arm or left_arm: input was set to " + arm)

        # Copy the measured position over to the commanded position so that they match
        msg.commanded_joint_position = msg.measured_joint_position

        # Note that torques and velocities are not implemented anywhere in any meaningful sense.
        # These are just here to have valid and known data
        for joint in MinimalFakeArmInterface.joint_names:
            setattr(msg.measured_joint_velocity, joint, 0.0)
            setattr(msg.measured_joint_torque, joint, 0.0)
            setattr(msg.estimated_external_torque, joint, 0.0)

        # Nothing in cartesian mode is implemented. These are just dummy values that are valid for their type
        for field in MinimalFakeArmInterface.cartesian_field_names:
            setattr(msg.measured_cartesian_pose_abc, field, 0.0)
            setattr(msg.commanded_cartesian_pose_abc, field, 0.0)

        msg.measured_cartesian_pose.position.x = 0.0
        msg.measured_cartesian_pose.position.y = 0.0
        msg.measured_cartesian_pose.position.z = 0.0
        msg.measured_cartesian_pose.orientation.x = 0.0
        msg.measured_cartesian_pose.orientation.y = 0.0
        msg.measured_cartesian_pose.orientation.z = 0.0
        msg.measured_cartesian_pose.orientation.w = 1.0

        msg.commanded_cartesian_pose = msg.measured_cartesian_pose

        msg.active_control_mode.mode = ControlMode.JOINT_POSITION

        return msg
