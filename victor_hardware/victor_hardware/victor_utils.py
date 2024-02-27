from enum import Enum, auto

from typing import List, Sequence

from victor_hardware_interfaces.msg import ControlMode, ControlModeParameters, Robotiq3FingerStatus, \
    Robotiq3FingerCommand, JointValueQuantity


class Stiffness(Enum):
    STIFF = 0  # we rely on this being 0 for indexing reasons
    MEDIUM = auto()
    SOFT = auto()


def get_control_mode_params(control_mode: ControlMode, stiffness=Stiffness.MEDIUM, vel=0.1, accel=0.1):
    if control_mode == ControlMode.JOINT_POSITION:
        return get_joint_position_params(vel, accel)
    elif control_mode == ControlMode.JOINT_IMPEDANCE:
        return get_joint_impedance_params(stiffness, vel, accel)
    elif control_mode == ControlMode.CARTESIAN_IMPEDANCE:
        # the semantics of this vel (even a value of 2.0 is slow) and moveit (expects [0, 1]) is very different
        return get_cartesian_impedance_params(velocity=vel * 40)
    elif control_mode == ControlMode.CARTESIAN_POSE:
        raise NotImplementedError("Cartesian Mode not yet implemented")
    else:
        raise NotImplementedError(f"Unknown control mode requested: {control_mode}")


def get_joint_position_params(vel, accel):
    new_control_mode = ControlModeParameters()
    new_control_mode.control_mode.mode = ControlMode.JOINT_POSITION
    new_control_mode.joint_path_execution_params.joint_relative_velocity = vel
    new_control_mode.joint_path_execution_params.joint_relative_acceleration = accel
    new_control_mode.joint_path_execution_params.override_joint_acceleration = 0.0
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
        new_control_mode.joint_impedance_params.joint_stiffness.joint_1 = 125.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_2 = 125.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_3 = 50.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_4 = 50.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_5 = 25.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_6 = 25.0
        new_control_mode.joint_impedance_params.joint_stiffness.joint_7 = 10.0

    else:
        print("Unknown stiffness for Joint Impedance")
        assert False

    return new_control_mode


def get_cartesian_impedance_params(velocity=75.):
    """
    Returns predefined cartesian impedance parameters. Uniform across XYZ and RPY
    """
    # TODO create specific configs for things like XY planar motion
    new_control_mode = ControlModeParameters()
    new_control_mode.control_mode.mode = ControlMode.CARTESIAN_IMPEDANCE
    new_control_mode.cartesian_path_execution_params.max_velocity.x = velocity
    new_control_mode.cartesian_path_execution_params.max_velocity.y = velocity
    new_control_mode.cartesian_path_execution_params.max_velocity.z = velocity
    new_control_mode.cartesian_path_execution_params.max_velocity.a = velocity * 0.25
    new_control_mode.cartesian_path_execution_params.max_velocity.b = velocity * 0.25
    new_control_mode.cartesian_path_execution_params.max_velocity.c = velocity * 0.25
    new_control_mode.cartesian_path_execution_params.max_nullspace_velocity = 750.0
    new_control_mode.cartesian_path_execution_params.max_acceleration.x = 0.1
    new_control_mode.cartesian_path_execution_params.max_acceleration.y = 0.1
    new_control_mode.cartesian_path_execution_params.max_acceleration.z = 0.1
    new_control_mode.cartesian_path_execution_params.max_acceleration.a = 0.1
    new_control_mode.cartesian_path_execution_params.max_acceleration.b = 0.1
    new_control_mode.cartesian_path_execution_params.max_acceleration.c = 0.1
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


def gripper_status_to_list(gripper_status: Robotiq3FingerStatus) -> List[float]:
    return [gripper_status.finger_a_status.position,
            gripper_status.finger_b_status.position,
            gripper_status.finger_c_status.position,
            gripper_status.scissor_status.position]


def list_to_gripper_status(gripper_status_list: Sequence[float]) -> Robotiq3FingerStatus:
    assert len(gripper_status_list) == 4, "gripper status list must be length 4"
    gripper_status = Robotiq3FingerStatus()
    gripper_status.finger_a_status.position = gripper_status_list[0]
    gripper_status.finger_b_status.position = gripper_status_list[1]
    gripper_status.finger_c_status.position = gripper_status_list[2]
    gripper_status.scissor_status.position = gripper_status_list[3]
    return gripper_status


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


def is_gripper_closed(status: Robotiq3FingerStatus):
    finger_a_closed = status.finger_a_status.position > 0.5 - 1e-2
    finger_b_closed = status.finger_b_status.position > 0.5 - 1e-2
    finger_c_closed = status.finger_c_status.position > 0.5 - 1e-2
    return finger_a_closed and finger_b_closed and finger_c_closed


def get_gripper_closed_fraction_msg(position: float):
    """
    Args:
        position: 0.0 is open, 1.0 is closed
    """
    msg = default_gripper_command()
    msg.finger_a_command.position = position
    msg.finger_b_command.position = position
    msg.finger_c_command.position = position
    return msg
