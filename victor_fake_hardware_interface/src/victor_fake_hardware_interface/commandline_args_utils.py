import argparse

from victor_hardware_interface_msgs.msg import ControlMode

control_mode_strings = ['JOINT_POSITION',
                        'JOINT_IMPEDANCE',
                        'CARTESIAN_POSE',
                        'CARTESIAN_IMPEDANCE']


def control_mode_arg(name: str):
    """
    Args:
        name: string that should match one of the control mode constants

    Returns: control mode constant

    """
    if hasattr(ControlMode, name):
        return getattr(ControlMode, name)
    else:
        raise argparse.ArgumentTypeError(f"invalid control mode {name}, choices are {control_mode_strings}")
