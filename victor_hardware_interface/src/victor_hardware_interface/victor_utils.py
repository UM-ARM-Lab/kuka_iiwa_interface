#! /usr/bin/env python

from enum import Enum

import rospy
from victor_hardware_interface_msgs.msg import *
from victor_hardware_interface_msgs.srv import *


class Stiffness(Enum):
    STIFF = 1
    MEDIUM = 3
    SOFT = 2


def get_control_mode_params(control_mode: ControlMode, stiffness=Stiffness.MEDIUM, vel=0.1, accel=0.1):
    if control_mode == ControlMode.JOINT_POSITION:
        return get_joint_position_params(vel, accel)
    elif control_mode == ControlMode.JOINT_IMPEDANCE:
        return get_joint_impedance_params(stiffness, vel, accel)
    elif control_mode == ControlMode.CARTESIAN_IMPEDANCE:
        return get_cartesian_impedance_params(velocity=vel)
    elif control_mode == ControlMode.CARTESIAN_POSE:
        raise NotImplementedError("Cartesian Mode not yet implemented")
    else:
        raise NotImplementedError(f"Unknown control mode requested: {control_mode}")


def set_control_mode(control_mode, arm, stiffness=Stiffness.MEDIUM, vel=0.1, accel=0.1):
    """
    Sets Victor's control mode.

    Parameters:
    control_mode (ControlMode): The control mode to enter
    arm (string):               The name of the arm: "right_arm" or "left_arm"
    stiffness (Stiffness):      For impedance modes, uses a set of stiffness values
    """
    new_control_mode = get_control_mode_params(control_mode, stiffness, vel, accel)
    result = send_new_control_mode(arm, new_control_mode)
    if not result.success:
        rospy.logerr("Failed to switch to control mode: " + str(control_mode))
    return result


def send_new_control_mode(arm, msg):
    # TODO: Consider removing the forced global namespace in the future
    send_new_control_mode_srv = rospy.ServiceProxy("victor/" + arm + "/set_control_mode_service",
                                                   SetControlMode)
    return send_new_control_mode_srv(msg)


def get_joint_position_params(vel, accel):
    new_control_mode = ControlModeParameters()
    new_control_mode.control_mode.mode = ControlMode.JOINT_POSITION
    new_control_mode.joint_path_execution_params.joint_relative_velocity = vel
    new_control_mode.joint_path_execution_params.joint_relative_acceleration = accel
    return new_control_mode


def get_joint_impedance_params(stiffness, vel=0.1, accel=0.1):
    """
    Returns predefined joint impedance stiffness parameters for a few different stiffnesses
    """
    new_control_mode = ControlModeParameters()
    new_control_mode.control_mode.mode = ControlMode.JOINT_IMPEDANCE

    if stiffness == Stiffness.STIFF:
        new_control_mode.joint_path_execution_params.joint_relative_velocity = vel
        new_control_mode.joint_path_execution_params.joint_relative_acceleration = accel

        new_control_mode.joint_impedance_params.joint_damping.joint_1 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_2 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_3 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_4 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_5 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_6 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_7 = 0.7
        new_control_mode.joint_impedance_params.joint_stiffness.joint_1 = 600.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_2 = 600.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_3 = 300.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_4 = 300.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_5 = 100.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_6 = 100.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_7 = 50.0

    elif stiffness == Stiffness.MEDIUM:
        new_control_mode.joint_path_execution_params.joint_relative_velocity = vel
        new_control_mode.joint_path_execution_params.joint_relative_acceleration = accel

        new_control_mode.joint_impedance_params.joint_damping.joint_1 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_2 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_3 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_4 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_5 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_6 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_7 = 0.7
        new_control_mode.joint_impedance_params.joint_stiffness.joint_1 = 200.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_2 = 200.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_3 = 100.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_4 = 100.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_5 = 50.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_6 = 50.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_7 = 20.0

    elif stiffness == Stiffness.SOFT:
        new_control_mode.joint_path_execution_params.joint_relative_velocity = vel
        new_control_mode.joint_path_execution_params.joint_relative_acceleration = accel

        new_control_mode.joint_impedance_params.joint_damping.joint_1 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_2 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_3 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_4 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_5 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_6 = 0.7
        new_control_mode.joint_impedance_params.joint_damping.joint_7 = 0.7
        new_control_mode.joint_impedance_params.joint_stiffness.joint_1 = 10.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_2 = 10.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_3 = 5.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_4 = 5.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_5 = 3.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_6 = 3.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_7 = 1.0

    else:
        rospy.logerr("Unknown stiffness for Joint Impedance")
        assert False

    return new_control_mode


def get_cartesian_impedance_params(velocity=2.5):
    """
    Returns predefined cartesian impedance parameters
    """
    # TODO allow sub-modes such as xy planar motion
    new_control_mode = ControlModeParameters()
    new_control_mode.control_mode.mode = ControlMode.CARTESIAN_IMPEDANCE
    new_control_mode.cartesian_path_execution_params.max_velocity.x = velocity
    new_control_mode.cartesian_path_execution_params.max_velocity.y = velocity
    new_control_mode.cartesian_path_execution_params.max_velocity.z = velocity
    new_control_mode.cartesian_path_execution_params.max_velocity.a = velocity * 0.25
    new_control_mode.cartesian_path_execution_params.max_velocity.b = velocity * 0.25
    new_control_mode.cartesian_path_execution_params.max_velocity.c = velocity * 0.25
    new_control_mode.cartesian_path_execution_params.max_nullspace_velocity = 750.0
    new_control_mode.cartesian_path_execution_params.max_acceleration.x = 100.0
    new_control_mode.cartesian_path_execution_params.max_acceleration.y = 100.0
    new_control_mode.cartesian_path_execution_params.max_acceleration.z = 100.0
    new_control_mode.cartesian_path_execution_params.max_acceleration.a = 100.0
    new_control_mode.cartesian_path_execution_params.max_acceleration.b = 100.0
    new_control_mode.cartesian_path_execution_params.max_acceleration.c = 100.0
    new_control_mode.cartesian_path_execution_params.max_nullspace_acceleration = 100.0
    new_control_mode.cartesian_impedance_params.cartesian_damping.x = 0.25
    new_control_mode.cartesian_impedance_params.cartesian_damping.y = 0.25
    new_control_mode.cartesian_impedance_params.cartesian_damping.z = 0.25
    new_control_mode.cartesian_impedance_params.cartesian_damping.a = 0.25
    new_control_mode.cartesian_impedance_params.cartesian_damping.b = 0.25
    new_control_mode.cartesian_impedance_params.cartesian_damping.c = 0.25
    new_control_mode.cartesian_impedance_params.nullspace_damping = 0.5
    new_control_mode.cartesian_impedance_params.cartesian_stiffness.x = 5000.0
    new_control_mode.cartesian_impedance_params.cartesian_stiffness.y = 5000.0
    new_control_mode.cartesian_impedance_params.cartesian_stiffness.z = 5000.0
    new_control_mode.cartesian_impedance_params.cartesian_stiffness.a = 300.0
    new_control_mode.cartesian_impedance_params.cartesian_stiffness.b = 300.0
    new_control_mode.cartesian_impedance_params.cartesian_stiffness.c = 300.0
    new_control_mode.cartesian_impedance_params.nullspace_stiffness = 100.0
    new_control_mode.cartesian_control_mode_limits.max_path_deviation.x = 10000000.0
    new_control_mode.cartesian_control_mode_limits.max_path_deviation.y = 10000000.0
    new_control_mode.cartesian_control_mode_limits.max_path_deviation.z = 10000000.0
    new_control_mode.cartesian_control_mode_limits.max_path_deviation.a = 10000000.0
    new_control_mode.cartesian_control_mode_limits.max_path_deviation.b = 10000000.0
    new_control_mode.cartesian_control_mode_limits.max_path_deviation.c = 10000000.0
    new_control_mode.cartesian_control_mode_limits.max_cartesian_velocity.x = velocity
    new_control_mode.cartesian_control_mode_limits.max_cartesian_velocity.y = velocity
    new_control_mode.cartesian_control_mode_limits.max_cartesian_velocity.z = velocity
    new_control_mode.cartesian_control_mode_limits.max_cartesian_velocity.a = velocity * 2.0 * 0.25
    new_control_mode.cartesian_control_mode_limits.max_cartesian_velocity.b = velocity * 2.0 * 0.25
    new_control_mode.cartesian_control_mode_limits.max_cartesian_velocity.c = velocity * 2.0 * 0.25
    new_control_mode.cartesian_control_mode_limits.max_control_force.x = 20.0
    new_control_mode.cartesian_control_mode_limits.max_control_force.y = 20.0
    new_control_mode.cartesian_control_mode_limits.max_control_force.z = 20.0
    new_control_mode.cartesian_control_mode_limits.max_control_force.a = 20.0
    new_control_mode.cartesian_control_mode_limits.max_control_force.b = 20.0
    new_control_mode.cartesian_control_mode_limits.max_control_force.c = 20.0
    new_control_mode.cartesian_control_mode_limits.stop_on_max_control_force = False
    return new_control_mode


def jvq_to_list(jvq):
    """Turns a joint value quantity into a list
    Parameters:
    jvq JointValueQuantity

    Return:
    list with 7 elements
    """
    return [getattr(jvq, 'joint_' + str(num)) for num in range(1, 8)]


def list_to_jvq(quantity_list):
    """Turns a list of 7 numbers into a joint value quantity
    Parameters:
    list with 7 elements

    Return:
    jvq JointValueQuantity
    """
    assert (len(quantity_list) == 7)
    jvq = JointValueQuantity()
    for i in range(7):
        setattr(jvq, 'joint_' + str(i + 1), quantity_list[i])
    return jvq


def default_gripper_command():
    cmd = Robotiq3FingerCommand()
    cmd.finger_a_command.speed = 0.5
    cmd.finger_b_command.speed = 0.5
    cmd.finger_c_command.speed = 0.5
    cmd.scissor_command.speed = 1.0

    cmd.finger_a_command.force = 1.0
    cmd.finger_b_command.force = 1.0
    cmd.finger_c_command.force = 1.0
    cmd.scissor_command.force = 1.0

    cmd.scissor_command.position = 1.0
    return cmd
