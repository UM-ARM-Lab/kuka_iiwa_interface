import numpy as np
import ros2_numpy
from ros2_numpy.registry import converts_to_numpy, converts_from_numpy

from victor_hardware_interfaces.msg import (
    JointValueQuantity,
    CartesianValueQuantity,
    ControlMode,
    JointImpedanceParameters,
    CartesianImpedanceParameters,
    JointPathExecutionParameters,
    CartesianPathExecutionParameters,
    CartesianControlModeLimits,
    ControlModeParameters,
    MotionStatus,
    Robotiq3FingerActuatorCommand,
    Robotiq3FingerActuatorStatus,
    Robotiq3FingerObjectStatus,
    Robotiq3FingerCommand,
    Robotiq3FingerStatus
)

from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import Header

# ===== FUNDAMENTAL MESSAGE TYPES =====

# JointValueQuantity conversion (already implemented)
@converts_to_numpy(JointValueQuantity)
def joint_value_to_numpy(msg):
    return np.array([
        msg.joint_1, msg.joint_2, msg.joint_3, msg.joint_4,
        msg.joint_5, msg.joint_6, msg.joint_7
    ])

@converts_from_numpy(JointValueQuantity)
def joint_value_from_numpy(array):
    jvq = JointValueQuantity()
    jvq.joint_1, jvq.joint_2, jvq.joint_3, jvq.joint_4, jvq.joint_5, jvq.joint_6, jvq.joint_7 = array
    return jvq

# CartesianValueQuantity conversion
@converts_to_numpy(CartesianValueQuantity)
def cartesian_value_to_numpy(msg):
    return np.array([msg.x, msg.y, msg.z, msg.a, msg.b, msg.c])

@converts_from_numpy(CartesianValueQuantity)
def cartesian_value_from_numpy(array):
    cvq = CartesianValueQuantity()
    cvq.x, cvq.y, cvq.z, cvq.a, cvq.b, cvq.c = array
    return cvq

# ControlMode conversion
@converts_to_numpy(ControlMode)
def control_mode_to_numpy(msg):
    return np.array([msg.mode], dtype=np.uint8)

@converts_from_numpy(ControlMode)
def control_mode_from_numpy(array):
    cm = ControlMode()
    cm.mode = int(array[0])
    return cm

# ===== ROBOTIQ MESSAGE TYPES =====

# Robotiq3FingerActuatorCommand conversion
@converts_to_numpy(Robotiq3FingerActuatorCommand)
def robotiq_actuator_command_to_numpy(msg):
    return np.array([msg.position, msg.speed, msg.force])

@converts_from_numpy(Robotiq3FingerActuatorCommand)
def robotiq_actuator_command_from_numpy(array):
    cmd = Robotiq3FingerActuatorCommand()
    cmd.position, cmd.speed, cmd.force = array
    return cmd

# Robotiq3FingerActuatorStatus conversion
@converts_to_numpy(Robotiq3FingerActuatorStatus)
def robotiq_actuator_status_to_numpy(msg):
    return np.array([msg.position_request, msg.position, msg.current])

@converts_from_numpy(Robotiq3FingerActuatorStatus)
def robotiq_actuator_status_from_numpy(array):
    status = Robotiq3FingerActuatorStatus()
    status.position_request, status.position, status.current = array
    return status

# Robotiq3FingerObjectStatus conversion
@converts_to_numpy(Robotiq3FingerObjectStatus)
def robotiq_object_status_to_numpy(msg):
    return np.array([msg.status], dtype=np.uint8)

@converts_from_numpy(Robotiq3FingerObjectStatus)
def robotiq_object_status_from_numpy(array):
    status = Robotiq3FingerObjectStatus()
    status.status = int(array[0])
    return status

# ===== COMPOUND MESSAGE TYPES =====

# JointImpedanceParameters conversion
@converts_to_numpy(JointImpedanceParameters)
def joint_impedance_params_to_numpy(msg):
    joint_stiffness = ros2_numpy.numpify(msg.joint_stiffness)
    joint_damping = ros2_numpy.numpify(msg.joint_damping)
    return np.concatenate([joint_stiffness, joint_damping])

@converts_from_numpy(JointImpedanceParameters)
def joint_impedance_params_from_numpy(array):
    params = JointImpedanceParameters()
    mid = len(array) // 2
    params.joint_stiffness = ros2_numpy.msgify(JointValueQuantity, array[:mid])
    params.joint_damping = ros2_numpy.msgify(JointValueQuantity, array[mid:])
    return params

# CartesianImpedanceParameters conversion
@converts_to_numpy(CartesianImpedanceParameters)
def cartesian_impedance_params_to_numpy(msg):
    cartesian_stiffness = ros2_numpy.numpify(msg.cartesian_stiffness)
    cartesian_damping = ros2_numpy.numpify(msg.cartesian_damping)
    return np.concatenate([
        cartesian_stiffness, 
        np.array([msg.nullspace_stiffness]), 
        cartesian_damping, 
        np.array([msg.nullspace_damping])
    ])

@converts_from_numpy(CartesianImpedanceParameters)
def cartesian_impedance_params_from_numpy(array):
    params = CartesianImpedanceParameters()
    params.cartesian_stiffness = ros2_numpy.msgify(CartesianValueQuantity, array[:6])
    params.nullspace_stiffness = array[6]
    params.cartesian_damping = ros2_numpy.msgify(CartesianValueQuantity, array[7:13])
    params.nullspace_damping = array[13]
    return params

# JointPathExecutionParameters conversion
@converts_to_numpy(JointPathExecutionParameters)
def joint_path_params_to_numpy(msg):
    return np.array([
        msg.joint_relative_velocity,
        msg.joint_relative_acceleration,
        msg.override_joint_acceleration
    ])

@converts_from_numpy(JointPathExecutionParameters)
def joint_path_params_from_numpy(array):
    params = JointPathExecutionParameters()
    params.joint_relative_velocity = array[0]
    params.joint_relative_acceleration = array[1]
    params.override_joint_acceleration = array[2]
    return params

# CartesianPathExecutionParameters conversion
@converts_to_numpy(CartesianPathExecutionParameters)
def cartesian_path_params_to_numpy(msg):
    max_velocity = ros2_numpy.numpify(msg.max_velocity)
    max_acceleration = ros2_numpy.numpify(msg.max_acceleration)
    return np.concatenate([
        max_velocity,
        max_acceleration,
        [msg.max_nullspace_velocity, msg.max_nullspace_acceleration]
    ])

@converts_from_numpy(CartesianPathExecutionParameters)
def cartesian_path_params_from_numpy(array):
    params = CartesianPathExecutionParameters()
    params.max_velocity = ros2_numpy.msgify(CartesianValueQuantity, array[:6])
    params.max_acceleration = ros2_numpy.msgify(CartesianValueQuantity, array[6:12])
    params.max_nullspace_velocity = array[12]
    params.max_nullspace_acceleration = array[13]
    return params

# CartesianControlModeLimits conversion
@converts_to_numpy(CartesianControlModeLimits)
def cartesian_control_limits_to_numpy(msg):
    max_path_deviation = ros2_numpy.numpify(msg.max_path_deviation)
    max_cartesian_velocity = ros2_numpy.numpify(msg.max_cartesian_velocity)
    max_control_force = ros2_numpy.numpify(msg.max_control_force)
    return np.concatenate([
        max_path_deviation,
        max_cartesian_velocity,
        max_control_force,
        [float(msg.stop_on_max_control_force)]
    ])

@converts_from_numpy(CartesianControlModeLimits)
def cartesian_control_limits_from_numpy(array):
    limits = CartesianControlModeLimits()
    limits.max_path_deviation = ros2_numpy.msgify(CartesianValueQuantity, array[:6])
    limits.max_cartesian_velocity = ros2_numpy.msgify(CartesianValueQuantity, array[6:12])
    limits.max_control_force = ros2_numpy.msgify(CartesianValueQuantity, array[12:18])
    limits.stop_on_max_control_force = bool(array[18])
    return limits

# Robotiq3FingerCommand conversion
@converts_to_numpy(Robotiq3FingerCommand)
def robotiq_command_to_numpy(msg):
    finger_a = ros2_numpy.numpify(msg.finger_a_command)
    finger_b = ros2_numpy.numpify(msg.finger_b_command)
    finger_c = ros2_numpy.numpify(msg.finger_c_command)
    scissor = ros2_numpy.numpify(msg.scissor_command)
    return np.concatenate([finger_a, finger_b, finger_c, scissor])

@converts_from_numpy(Robotiq3FingerCommand)
def robotiq_command_from_numpy(array):
    cmd = Robotiq3FingerCommand()
    cmd.finger_a_command = ros2_numpy.msgify(Robotiq3FingerActuatorCommand, array[:3])
    cmd.finger_b_command = ros2_numpy.msgify(Robotiq3FingerActuatorCommand, array[3:6])
    cmd.finger_c_command = ros2_numpy.msgify(Robotiq3FingerActuatorCommand, array[6:9])
    cmd.scissor_command = ros2_numpy.msgify(Robotiq3FingerActuatorCommand, array[9:12])
    return cmd

# Robotiq3FingerStatus conversion
@converts_to_numpy(Robotiq3FingerStatus)
def robotiq_status_to_numpy(msg):
    finger_a_status = ros2_numpy.numpify(msg.finger_a_status)
    finger_b_status = ros2_numpy.numpify(msg.finger_b_status)
    finger_c_status = ros2_numpy.numpify(msg.finger_c_status)
    scissor_status = ros2_numpy.numpify(msg.scissor_status)
    
    finger_a_obj = ros2_numpy.numpify(msg.finger_a_object_status)
    finger_b_obj = ros2_numpy.numpify(msg.finger_b_object_status)
    finger_c_obj = ros2_numpy.numpify(msg.finger_c_object_status)
    scissor_obj = ros2_numpy.numpify(msg.scissor_object_status)
    
    status_values = np.array([
        msg.initialization_status,
        msg.gripper_action_status,
        msg.gripper_system_status,
        msg.gripper_motion_status,
        msg.gripper_fault_status
    ], dtype=np.uint8)
    
    return np.concatenate([
        finger_a_status, finger_b_status, finger_c_status, scissor_status,
        finger_a_obj, finger_b_obj, finger_c_obj, scissor_obj,
        status_values.astype(np.float64)
    ])

@converts_from_numpy(Robotiq3FingerStatus)
def robotiq_status_from_numpy(array):
    status = Robotiq3FingerStatus()
    
    # Actuator statuses (3 values each)
    status.finger_a_status = ros2_numpy.msgify(Robotiq3FingerActuatorStatus, array[0:3])
    status.finger_b_status = ros2_numpy.msgify(Robotiq3FingerActuatorStatus, array[3:6])
    status.finger_c_status = ros2_numpy.msgify(Robotiq3FingerActuatorStatus, array[6:9])
    status.scissor_status = ros2_numpy.msgify(Robotiq3FingerActuatorStatus, array[9:12])
    
    # Object statuses (1 value each)
    status.finger_a_object_status = ros2_numpy.msgify(Robotiq3FingerObjectStatus, array[12:13])
    status.finger_b_object_status = ros2_numpy.msgify(Robotiq3FingerObjectStatus, array[13:14])
    status.finger_c_object_status = ros2_numpy.msgify(Robotiq3FingerObjectStatus, array[14:15])
    status.scissor_object_status = ros2_numpy.msgify(Robotiq3FingerObjectStatus, array[15:16])
    
    # Status values
    status.initialization_status = int(array[16])
    status.gripper_action_status = int(array[17])
    status.gripper_system_status = int(array[18])
    status.gripper_motion_status = int(array[19])
    status.gripper_fault_status = int(array[20])
    
# ===== GEOMETRY AND STD MESSAGE TYPES =====

# Pose conversion (geometry_msgs/Pose)
@converts_to_numpy(Pose)
def pose_to_numpy(msg):
    return np.array([
        msg.position.x, msg.position.y, msg.position.z,
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
    ])

@converts_from_numpy(Pose)
def pose_from_numpy(array):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = array[0:3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = array[3:7]
    return pose

# TransformStamped conversion (geometry_msgs/TransformStamped)
@converts_to_numpy(TransformStamped)
def transform_stamped_to_numpy(msg):
    return np.array([
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z,
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w
    ])

@converts_from_numpy(TransformStamped)
def transform_stamped_from_numpy(array):
    ts = TransformStamped()
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = array[0:3]
    ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = array[3:7]
    return ts

# Header conversion (std_msgs/Header) - simplified version with just timestamp
@converts_to_numpy(Header)
def header_to_numpy(msg):
    return np.array([msg.stamp.sec, msg.stamp.nanosec], dtype=np.float64)

@converts_from_numpy(Header)
def header_from_numpy(array):
    header = Header()
    header.stamp.sec = int(array[0])
    header.stamp.nanosec = int(array[1])
    return header

# MotionStatus conversion
@converts_to_numpy(MotionStatus)
def motion_status_to_numpy(msg):
    measured_joint_pos = ros2_numpy.numpify(msg.measured_joint_position)
    commanded_joint_pos = ros2_numpy.numpify(msg.commanded_joint_position)
    measured_joint_vel = ros2_numpy.numpify(msg.measured_joint_velocity)
    measured_joint_torque = ros2_numpy.numpify(msg.measured_joint_torque)
    estimated_external_torque = ros2_numpy.numpify(msg.estimated_external_torque)
    estimated_external_wrench = ros2_numpy.numpify(msg.estimated_external_wrench)
    measured_cartesian_pose_abc = ros2_numpy.numpify(msg.measured_cartesian_pose_abc)
    commanded_cartesian_pose_abc = ros2_numpy.numpify(msg.commanded_cartesian_pose_abc)
    measured_cartesian_pose = ros2_numpy.numpify(msg.measured_cartesian_pose)
    commanded_cartesian_pose = ros2_numpy.numpify(msg.commanded_cartesian_pose)
    active_control_mode = ros2_numpy.numpify(msg.active_control_mode)
    header = ros2_numpy.numpify(msg.header)
    
    return np.concatenate([
        measured_joint_pos, commanded_joint_pos, measured_joint_vel,
        measured_joint_torque, estimated_external_torque, estimated_external_wrench,
        measured_cartesian_pose_abc, commanded_cartesian_pose_abc,
        measured_cartesian_pose, commanded_cartesian_pose,
        active_control_mode, header
    ])

@converts_from_numpy(MotionStatus)
def motion_status_from_numpy(array):
    status = MotionStatus()
    
    # Joint quantities (7 values each)
    status.measured_joint_position = ros2_numpy.msgify(JointValueQuantity, array[0:7])
    status.commanded_joint_position = ros2_numpy.msgify(JointValueQuantity, array[7:14])
    status.measured_joint_velocity = ros2_numpy.msgify(JointValueQuantity, array[14:21])
    status.measured_joint_torque = ros2_numpy.msgify(JointValueQuantity, array[21:28])
    status.estimated_external_torque = ros2_numpy.msgify(JointValueQuantity, array[28:35])
    
    # Cartesian quantities (6 values each)
    status.estimated_external_wrench = ros2_numpy.msgify(CartesianValueQuantity, array[35:41])
    status.measured_cartesian_pose_abc = ros2_numpy.msgify(CartesianValueQuantity, array[41:47])
    status.commanded_cartesian_pose_abc = ros2_numpy.msgify(CartesianValueQuantity, array[47:53])
    
    # Pose quantities (7 values each)
    status.measured_cartesian_pose = ros2_numpy.msgify(Pose, array[53:60])
    status.commanded_cartesian_pose = ros2_numpy.msgify(Pose, array[60:67])
    
    # Control mode (1 value)
    status.active_control_mode = ros2_numpy.msgify(ControlMode, array[67:68])
    
    # Header (2 values)
    status.header = ros2_numpy.msgify(Header, array[68:70])
    
    return status

# ===== INDICES CACHING AND RETRIEVAL SYSTEM =====

# Cache for storing indices mappings
_INDICES_CACHE = {}

def _build_indices_cache_for_type(msg_type):
    """Build indices cache for a message type by analyzing its numpy conversion structure."""
    if msg_type in _INDICES_CACHE:
        return _INDICES_CACHE[msg_type]
    
    cache = {}
    current_index = 0
    
    if msg_type == JointValueQuantity:
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        for i, joint in enumerate(joint_names):
            cache[joint] = slice(current_index + i, current_index + i + 1)
        current_index += 7
        
    elif msg_type == CartesianValueQuantity:
        cart_names = ['x', 'y', 'z', 'a', 'b', 'c']
        for i, cart in enumerate(cart_names):
            cache[cart] = slice(current_index + i, current_index + i + 1)
        current_index += 6
        
    elif msg_type == ControlMode:
        cache['mode'] = slice(0, 1)
        current_index += 1
        
    elif msg_type == Pose:
        pose_names = ['position.x', 'position.y', 'position.z', 
                     'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w']
        for i, pose_field in enumerate(pose_names):
            cache[pose_field] = slice(current_index + i, current_index + i + 1)
        current_index += 7
        
    elif msg_type == Header:
        cache['stamp.sec'] = slice(0, 1)
        cache['stamp.nanosec'] = slice(1, 2)
        current_index += 2
        
    elif msg_type == Robotiq3FingerActuatorCommand:
        cache['position'] = slice(0, 1)
        cache['speed'] = slice(1, 2)
        cache['force'] = slice(2, 3)
        current_index += 3
        
    elif msg_type == Robotiq3FingerActuatorStatus:
        cache['position_request'] = slice(0, 1)
        cache['position'] = slice(1, 2)
        cache['current'] = slice(2, 3)
        current_index += 3
        
    elif msg_type == Robotiq3FingerObjectStatus:
        cache['status'] = slice(0, 1)
        current_index += 1
        
    elif msg_type == JointImpedanceParameters:
        # Joint stiffness (7 values)
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        for i, joint in enumerate(joint_names):
            cache[f'joint_stiffness.{joint}'] = slice(current_index + i, current_index + i + 1)
        current_index += 7
        # Joint damping (7 values)
        for i, joint in enumerate(joint_names):
            cache[f'joint_damping.{joint}'] = slice(current_index + i, current_index + i + 1)
        current_index += 7
        
    elif msg_type == CartesianImpedanceParameters:
        # Cartesian stiffness (6 values)
        cart_names = ['x', 'y', 'z', 'a', 'b', 'c']
        for i, cart in enumerate(cart_names):
            cache[f'cartesian_stiffness.{cart}'] = slice(current_index + i, current_index + i + 1)
        current_index += 6
        # Nullspace stiffness (1 value)
        cache['nullspace_stiffness'] = slice(current_index, current_index + 1)
        current_index += 1
        # Cartesian damping (6 values)
        for i, cart in enumerate(cart_names):
            cache[f'cartesian_damping.{cart}'] = slice(current_index + i, current_index + i + 1)
        current_index += 6
        # Nullspace damping (1 value)
        cache['nullspace_damping'] = slice(current_index, current_index + 1)
        current_index += 1
        
    elif msg_type == JointPathExecutionParameters:
        cache['joint_relative_velocity'] = slice(0, 1)
        cache['joint_relative_acceleration'] = slice(1, 2)
        cache['override_joint_acceleration'] = slice(2, 3)
        current_index += 3
        
    elif msg_type == CartesianPathExecutionParameters:
        # Max velocity (6 values)
        cart_names = ['x', 'y', 'z', 'a', 'b', 'c']
        for i, cart in enumerate(cart_names):
            cache[f'max_velocity.{cart}'] = slice(current_index + i, current_index + i + 1)
        current_index += 6
        # Max acceleration (6 values)
        for i, cart in enumerate(cart_names):
            cache[f'max_acceleration.{cart}'] = slice(current_index + i, current_index + i + 1)
        current_index += 6
        # Nullspace parameters
        cache['max_nullspace_velocity'] = slice(current_index, current_index + 1)
        cache['max_nullspace_acceleration'] = slice(current_index + 1, current_index + 2)
        current_index += 2
        
    elif msg_type == CartesianControlModeLimits:
        # Max path deviation (6 values)
        cart_names = ['x', 'y', 'z', 'a', 'b', 'c']
        for i, cart in enumerate(cart_names):
            cache[f'max_path_deviation.{cart}'] = slice(current_index + i, current_index + i + 1)
        current_index += 6
        # Max cartesian velocity (6 values)
        for i, cart in enumerate(cart_names):
            cache[f'max_cartesian_velocity.{cart}'] = slice(current_index + i, current_index + i + 1)
        current_index += 6
        # Max control force (6 values)
        for i, cart in enumerate(cart_names):
            cache[f'max_control_force.{cart}'] = slice(current_index + i, current_index + i + 1)
        current_index += 6
        # Stop on max control force
        cache['stop_on_max_control_force'] = slice(current_index, current_index + 1)
        current_index += 1
        
    elif msg_type == Robotiq3FingerCommand:
        # Finger commands (3 values each)
        fingers = ['finger_a_command', 'finger_b_command', 'finger_c_command', 'scissor_command']
        actuator_fields = ['position', 'speed', 'force']
        for i, finger in enumerate(fingers):
            for j, field in enumerate(actuator_fields):
                idx = i * 3 + j
                cache[f'{finger}.{field}'] = slice(current_index + idx, current_index + idx + 1)
        current_index += 12
        
    elif msg_type == Robotiq3FingerStatus:
        # Actuator statuses (3 values each for 4 actuators)
        fingers = ['finger_a_status', 'finger_b_status', 'finger_c_status', 'scissor_status']
        status_fields = ['position_request', 'position', 'current']
        for i, finger in enumerate(fingers):
            for j, field in enumerate(status_fields):
                idx = i * 3 + j
                cache[f'{finger}.{field}'] = slice(current_index + idx, current_index + idx + 1)
        current_index += 12
        # Object statuses (1 value each for 4 actuators)
        obj_fingers = ['finger_a_object_status', 'finger_b_object_status', 'finger_c_object_status', 'scissor_object_status']
        for i, finger in enumerate(obj_fingers):
            cache[f'{finger}.status'] = slice(current_index + i, current_index + i + 1)
        current_index += 4
        # Status values (5 values)
        status_names = ['initialization_status', 'gripper_action_status', 'gripper_system_status', 
                       'gripper_motion_status', 'gripper_fault_status']
        for i, status_name in enumerate(status_names):
            cache[status_name] = slice(current_index + i, current_index + i + 1)
        current_index += 5
        
    elif msg_type == MotionStatus:
        # Joint quantities (7 values each)
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        joint_fields = ['measured_joint_position', 'commanded_joint_position', 'measured_joint_velocity',
                       'measured_joint_torque', 'estimated_external_torque']
        for i, field in enumerate(joint_fields):
            for j, joint in enumerate(joint_names):
                idx = i * 7 + j
                cache[f'{field}.{joint}'] = slice(current_index + idx, current_index + idx + 1)
        current_index += 35
        
        # Cartesian quantities (6 values each)
        cart_names = ['x', 'y', 'z', 'a', 'b', 'c']
        cart_fields = ['estimated_external_wrench', 'measured_cartesian_pose_abc', 'commanded_cartesian_pose_abc']
        for i, field in enumerate(cart_fields):
            for j, cart in enumerate(cart_names):
                idx = i * 6 + j
                cache[f'{field}.{cart}'] = slice(current_index + idx, current_index + idx + 1)
        current_index += 18
        
        # Pose quantities (7 values each)
        pose_names = ['position.x', 'position.y', 'position.z', 
                     'orientation.x', 'orientation.y', 'orientation.z', 'orientation.w']
        pose_fields = ['measured_cartesian_pose', 'commanded_cartesian_pose']
        for i, field in enumerate(pose_fields):
            for j, pose_field in enumerate(pose_names):
                idx = i * 7 + j
                cache[f'{field}.{pose_field}'] = slice(current_index + idx, current_index + idx + 1)
        current_index += 14
        
        # Control mode (1 value)
        cache['active_control_mode.mode'] = slice(current_index, current_index + 1)
        current_index += 1
        
        # Header (2 values)
        cache['header.stamp.sec'] = slice(current_index, current_index + 1)
        cache['header.stamp.nanosec'] = slice(current_index + 1, current_index + 2)
        current_index += 2
    
    _INDICES_CACHE[msg_type] = cache
    return cache

def indices_of(msg_type, field_path):
    """
    Return the numpy array indices for a specific field within a message type's numpy representation.
    
    Args:
        msg_type: The ROS message type (e.g., MotionStatus, JointValueQuantity)
        field_path: String path to the field (e.g., "measured_joint_position.joint_1")
    
    Returns:
        slice: A slice object representing the indices in the numpy array
        
    Example:
        >>> indices_of(MotionStatus, "measured_joint_position.joint_1")
        slice(0, 1, None)
    """
    cache = _build_indices_cache_for_type(msg_type)
    
    if field_path not in cache:
        available_fields = list(cache.keys())
        raise KeyError(f"Field '{field_path}' not found for type {msg_type.__name__}. "
                      f"Available fields: {available_fields}")
    
    return cache[field_path]

def get_field_value_from_numpy(numpy_array, msg_type, field_path):
    """
    Extract a specific field value from a numpy array representation of a ROS message.
    
    Args:
        numpy_array: The numpy array representation of the message
        msg_type: The ROS message type
        field_path: String path to the field
    
    Returns:
        numpy array or scalar: The value(s) at the specified field
    """
    indices = indices_of(msg_type, field_path)
    return numpy_array[indices]

def set_field_value_in_numpy(numpy_array, msg_type, field_path, value):
    """
    Set a specific field value in a numpy array representation of a ROS message.
    
    Args:
        numpy_array: The numpy array representation of the message (modified in-place)
        msg_type: The ROS message type
        field_path: String path to the field
        value: The value to set
    """
    indices = indices_of(msg_type, field_path)
    numpy_array[indices] = value

def _victor_msg_test_conversion():
    """Test all message type conversions to numpy and back."""
    print("Testing victor_hardware_interfaces message conversions...")
    
    # Helper function to compare floating point values
    def almost_equal(a, b, tolerance=1e-10):
        return abs(a - b) < tolerance
    
    # Test JointValueQuantity
    print("Testing JointValueQuantity...")
    original_jvq = JointValueQuantity()
    original_jvq.joint_1 = 1.1
    original_jvq.joint_2 = 2.2
    original_jvq.joint_3 = 3.3
    original_jvq.joint_4 = 4.4
    original_jvq.joint_5 = 5.5
    original_jvq.joint_6 = 6.6
    original_jvq.joint_7 = 7.7
    
    numpy_jvq = ros2_numpy.numpify(original_jvq)
    restored_jvq = ros2_numpy.msgify(JointValueQuantity, numpy_jvq)
    
    assert almost_equal(original_jvq.joint_1, restored_jvq.joint_1)
    assert almost_equal(original_jvq.joint_2, restored_jvq.joint_2)
    assert almost_equal(original_jvq.joint_3, restored_jvq.joint_3)
    assert almost_equal(original_jvq.joint_4, restored_jvq.joint_4)
    assert almost_equal(original_jvq.joint_5, restored_jvq.joint_5)
    assert almost_equal(original_jvq.joint_6, restored_jvq.joint_6)
    assert almost_equal(original_jvq.joint_7, restored_jvq.joint_7)
    print("✓ JointValueQuantity conversion test passed")
    
    # Test CartesianValueQuantity
    print("Testing CartesianValueQuantity...")
    original_cvq = CartesianValueQuantity()
    original_cvq.x = 1.0
    original_cvq.y = 2.0
    original_cvq.z = 3.0
    original_cvq.a = 0.1
    original_cvq.b = 0.2
    original_cvq.c = 0.3
    
    numpy_cvq = ros2_numpy.numpify(original_cvq)
    restored_cvq = ros2_numpy.msgify(CartesianValueQuantity, numpy_cvq)
    
    assert almost_equal(original_cvq.x, restored_cvq.x)
    assert almost_equal(original_cvq.y, restored_cvq.y)
    assert almost_equal(original_cvq.z, restored_cvq.z)
    assert almost_equal(original_cvq.a, restored_cvq.a)
    assert almost_equal(original_cvq.b, restored_cvq.b)
    assert almost_equal(original_cvq.c, restored_cvq.c)
    print("✓ CartesianValueQuantity conversion test passed")
    
    # Test ControlMode
    print("Testing ControlMode...")
    original_cm = ControlMode()
    original_cm.mode = ControlMode.CARTESIAN_IMPEDANCE
    
    numpy_cm = ros2_numpy.numpify(original_cm)
    restored_cm = ros2_numpy.msgify(ControlMode, numpy_cm)
    
    assert original_cm.mode == restored_cm.mode
    print("✓ ControlMode conversion test passed")
    
    # Test Robotiq3FingerActuatorCommand
    print("Testing Robotiq3FingerActuatorCommand...")
    original_cmd = Robotiq3FingerActuatorCommand()
    original_cmd.position = 0.5
    original_cmd.speed = 0.8
    original_cmd.force = 0.9
    
    numpy_cmd = ros2_numpy.numpify(original_cmd)
    restored_cmd = ros2_numpy.msgify(Robotiq3FingerActuatorCommand, numpy_cmd)
    
    assert almost_equal(original_cmd.position, restored_cmd.position)
    assert almost_equal(original_cmd.speed, restored_cmd.speed)
    assert almost_equal(original_cmd.force, restored_cmd.force)
    print("✓ Robotiq3FingerActuatorCommand conversion test passed")
    
    # Test Robotiq3FingerActuatorStatus
    print("Testing Robotiq3FingerActuatorStatus...")
    original_status = Robotiq3FingerActuatorStatus()
    original_status.position_request = 0.6
    original_status.position = 0.55
    original_status.current = 0.12
    
    numpy_status = ros2_numpy.numpify(original_status)
    restored_status = ros2_numpy.msgify(Robotiq3FingerActuatorStatus, numpy_status)
    
    assert almost_equal(original_status.position_request, restored_status.position_request)
    assert almost_equal(original_status.position, restored_status.position)
    assert almost_equal(original_status.current, restored_status.current)
    print("✓ Robotiq3FingerActuatorStatus conversion test passed")
    
    # Test Robotiq3FingerObjectStatus
    print("Testing Robotiq3FingerObjectStatus...")
    original_obj_status = Robotiq3FingerObjectStatus()
    original_obj_status.status = Robotiq3FingerObjectStatus.CONTACT_CLOSING
    
    numpy_obj_status = ros2_numpy.numpify(original_obj_status)
    restored_obj_status = ros2_numpy.msgify(Robotiq3FingerObjectStatus, numpy_obj_status)
    
    assert original_obj_status.status == restored_obj_status.status
    print("✓ Robotiq3FingerObjectStatus conversion test passed")
    
    # Test JointImpedanceParameters
    print("Testing JointImpedanceParameters...")
    original_jip = JointImpedanceParameters()
    # Set joint_stiffness
    original_jip.joint_stiffness = JointValueQuantity()
    original_jip.joint_stiffness.joint_1 = 100.0
    original_jip.joint_stiffness.joint_2 = 200.0
    original_jip.joint_stiffness.joint_3 = 300.0
    original_jip.joint_stiffness.joint_4 = 400.0
    original_jip.joint_stiffness.joint_5 = 500.0
    original_jip.joint_stiffness.joint_6 = 600.0
    original_jip.joint_stiffness.joint_7 = 700.0
    # Set joint_damping
    original_jip.joint_damping = JointValueQuantity()
    original_jip.joint_damping.joint_1 = 10.0
    original_jip.joint_damping.joint_2 = 20.0
    original_jip.joint_damping.joint_3 = 30.0
    original_jip.joint_damping.joint_4 = 40.0
    original_jip.joint_damping.joint_5 = 50.0
    original_jip.joint_damping.joint_6 = 60.0
    original_jip.joint_damping.joint_7 = 70.0
    
    numpy_jip = ros2_numpy.numpify(original_jip)
    restored_jip = ros2_numpy.msgify(JointImpedanceParameters, numpy_jip)
    
    assert almost_equal(original_jip.joint_stiffness.joint_1, restored_jip.joint_stiffness.joint_1)
    assert almost_equal(original_jip.joint_stiffness.joint_7, restored_jip.joint_stiffness.joint_7)
    assert almost_equal(original_jip.joint_damping.joint_1, restored_jip.joint_damping.joint_1)
    assert almost_equal(original_jip.joint_damping.joint_7, restored_jip.joint_damping.joint_7)
    print("✓ JointImpedanceParameters conversion test passed")
    
    # Test CartesianImpedanceParameters
    print("Testing CartesianImpedanceParameters...")
    original_cip = CartesianImpedanceParameters()
    # Set cartesian_stiffness
    original_cip.cartesian_stiffness = CartesianValueQuantity()
    original_cip.cartesian_stiffness.x = 1000.0
    original_cip.cartesian_stiffness.y = 2000.0
    original_cip.cartesian_stiffness.z = 3000.0
    original_cip.cartesian_stiffness.a = 100.0
    original_cip.cartesian_stiffness.b = 200.0
    original_cip.cartesian_stiffness.c = 300.0
    original_cip.nullspace_stiffness = 50.0
    # Set cartesian_damping
    original_cip.cartesian_damping = CartesianValueQuantity()
    original_cip.cartesian_damping.x = 10.0
    original_cip.cartesian_damping.y = 20.0
    original_cip.cartesian_damping.z = 30.0
    original_cip.cartesian_damping.a = 1.0
    original_cip.cartesian_damping.b = 2.0
    original_cip.cartesian_damping.c = 3.0
    original_cip.nullspace_damping = 5.0
    
    numpy_cip = ros2_numpy.numpify(original_cip)
    restored_cip = ros2_numpy.msgify(CartesianImpedanceParameters, numpy_cip)
    
    assert almost_equal(original_cip.cartesian_stiffness.x, restored_cip.cartesian_stiffness.x)
    assert almost_equal(original_cip.cartesian_stiffness.c, restored_cip.cartesian_stiffness.c)
    assert almost_equal(original_cip.nullspace_stiffness, restored_cip.nullspace_stiffness)
    assert almost_equal(original_cip.cartesian_damping.x, restored_cip.cartesian_damping.x)
    assert almost_equal(original_cip.cartesian_damping.c, restored_cip.cartesian_damping.c)
    assert almost_equal(original_cip.nullspace_damping, restored_cip.nullspace_damping)
    print("✓ CartesianImpedanceParameters conversion test passed")
    
    # Test JointPathExecutionParameters
    print("Testing JointPathExecutionParameters...")
    original_jpep = JointPathExecutionParameters()
    original_jpep.joint_relative_velocity = 0.5
    original_jpep.joint_relative_acceleration = 0.1
    original_jpep.override_joint_acceleration = 0.2
    
    numpy_jpep = ros2_numpy.numpify(original_jpep)
    restored_jpep = ros2_numpy.msgify(JointPathExecutionParameters, numpy_jpep)
    
    assert almost_equal(original_jpep.joint_relative_velocity, restored_jpep.joint_relative_velocity)
    assert almost_equal(original_jpep.joint_relative_acceleration, restored_jpep.joint_relative_acceleration)
    assert almost_equal(original_jpep.override_joint_acceleration, restored_jpep.override_joint_acceleration)
    print("✓ JointPathExecutionParameters conversion test passed")
    
    # Test CartesianPathExecutionParameters
    print("Testing CartesianPathExecutionParameters...")
    original_cpep = CartesianPathExecutionParameters()
    # Set max_velocity
    original_cpep.max_velocity = CartesianValueQuantity()
    original_cpep.max_velocity.x = 0.1
    original_cpep.max_velocity.y = 0.2
    original_cpep.max_velocity.z = 0.3
    original_cpep.max_velocity.a = 0.01
    original_cpep.max_velocity.b = 0.02
    original_cpep.max_velocity.c = 0.03
    # Set max_acceleration
    original_cpep.max_acceleration = CartesianValueQuantity()
    original_cpep.max_acceleration.x = 1.0
    original_cpep.max_acceleration.y = 2.0
    original_cpep.max_acceleration.z = 3.0
    original_cpep.max_acceleration.a = 0.1
    original_cpep.max_acceleration.b = 0.2
    original_cpep.max_acceleration.c = 0.3
    original_cpep.max_nullspace_velocity = 0.05
    original_cpep.max_nullspace_acceleration = 0.5
    
    numpy_cpep = ros2_numpy.numpify(original_cpep)
    restored_cpep = ros2_numpy.msgify(CartesianPathExecutionParameters, numpy_cpep)
    
    assert almost_equal(original_cpep.max_velocity.x, restored_cpep.max_velocity.x)
    assert almost_equal(original_cpep.max_velocity.c, restored_cpep.max_velocity.c)
    assert almost_equal(original_cpep.max_acceleration.x, restored_cpep.max_acceleration.x)
    assert almost_equal(original_cpep.max_acceleration.c, restored_cpep.max_acceleration.c)
    assert almost_equal(original_cpep.max_nullspace_velocity, restored_cpep.max_nullspace_velocity)
    assert almost_equal(original_cpep.max_nullspace_acceleration, restored_cpep.max_nullspace_acceleration)
    print("✓ CartesianPathExecutionParameters conversion test passed")
    
    # Test CartesianControlModeLimits
    print("Testing CartesianControlModeLimits...")
    original_ccml = CartesianControlModeLimits()
    # Set max_path_deviation
    original_ccml.max_path_deviation = CartesianValueQuantity()
    original_ccml.max_path_deviation.x = 0.01
    original_ccml.max_path_deviation.y = 0.02
    original_ccml.max_path_deviation.z = 0.03
    original_ccml.max_path_deviation.a = 0.001
    original_ccml.max_path_deviation.b = 0.002
    original_ccml.max_path_deviation.c = 0.003
    # Set max_cartesian_velocity
    original_ccml.max_cartesian_velocity = CartesianValueQuantity()
    original_ccml.max_cartesian_velocity.x = 0.5
    original_ccml.max_cartesian_velocity.y = 0.6
    original_ccml.max_cartesian_velocity.z = 0.7
    original_ccml.max_cartesian_velocity.a = 0.05
    original_ccml.max_cartesian_velocity.b = 0.06
    original_ccml.max_cartesian_velocity.c = 0.07
    # Set max_control_force
    original_ccml.max_control_force = CartesianValueQuantity()
    original_ccml.max_control_force.x = 100.0
    original_ccml.max_control_force.y = 200.0
    original_ccml.max_control_force.z = 300.0
    original_ccml.max_control_force.a = 10.0
    original_ccml.max_control_force.b = 20.0
    original_ccml.max_control_force.c = 30.0
    original_ccml.stop_on_max_control_force = True
    
    numpy_ccml = ros2_numpy.numpify(original_ccml)
    restored_ccml = ros2_numpy.msgify(CartesianControlModeLimits, numpy_ccml)
    
    assert almost_equal(original_ccml.max_path_deviation.x, restored_ccml.max_path_deviation.x)
    assert almost_equal(original_ccml.max_cartesian_velocity.y, restored_ccml.max_cartesian_velocity.y)
    assert almost_equal(original_ccml.max_control_force.z, restored_ccml.max_control_force.z)
    assert original_ccml.stop_on_max_control_force == restored_ccml.stop_on_max_control_force
    print("✓ CartesianControlModeLimits conversion test passed")
    
    # Test Robotiq3FingerCommand
    print("Testing Robotiq3FingerCommand...")
    original_r3fc = Robotiq3FingerCommand()
    # Set finger_a_command
    original_r3fc.finger_a_command = Robotiq3FingerActuatorCommand()
    original_r3fc.finger_a_command.position = 0.1
    original_r3fc.finger_a_command.speed = 0.2
    original_r3fc.finger_a_command.force = 0.3
    # Set finger_b_command
    original_r3fc.finger_b_command = Robotiq3FingerActuatorCommand()
    original_r3fc.finger_b_command.position = 0.4
    original_r3fc.finger_b_command.speed = 0.5
    original_r3fc.finger_b_command.force = 0.6
    # Set finger_c_command
    original_r3fc.finger_c_command = Robotiq3FingerActuatorCommand()
    original_r3fc.finger_c_command.position = 0.7
    original_r3fc.finger_c_command.speed = 0.8
    original_r3fc.finger_c_command.force = 0.9
    # Set scissor_command
    original_r3fc.scissor_command = Robotiq3FingerActuatorCommand()
    original_r3fc.scissor_command.position = 0.15
    original_r3fc.scissor_command.speed = 0.25
    original_r3fc.scissor_command.force = 0.35
    
    numpy_r3fc = ros2_numpy.numpify(original_r3fc)
    restored_r3fc = ros2_numpy.msgify(Robotiq3FingerCommand, numpy_r3fc)
    
    assert almost_equal(original_r3fc.finger_a_command.position, restored_r3fc.finger_a_command.position)
    assert almost_equal(original_r3fc.finger_b_command.speed, restored_r3fc.finger_b_command.speed)
    assert almost_equal(original_r3fc.finger_c_command.force, restored_r3fc.finger_c_command.force)
    assert almost_equal(original_r3fc.scissor_command.position, restored_r3fc.scissor_command.position)
    print("✓ Robotiq3FingerCommand conversion test passed")
    
    # Test Robotiq3FingerStatus
    print("Testing Robotiq3FingerStatus...")
    original_r3fs = Robotiq3FingerStatus()
    # Set actuator statuses
    original_r3fs.finger_a_status = Robotiq3FingerActuatorStatus()
    original_r3fs.finger_a_status.position_request = 0.1
    original_r3fs.finger_a_status.position = 0.09
    original_r3fs.finger_a_status.current = 0.05
    
    original_r3fs.finger_b_status = Robotiq3FingerActuatorStatus()
    original_r3fs.finger_b_status.position_request = 0.2
    original_r3fs.finger_b_status.position = 0.19
    original_r3fs.finger_b_status.current = 0.06
    
    original_r3fs.finger_c_status = Robotiq3FingerActuatorStatus()
    original_r3fs.finger_c_status.position_request = 0.3
    original_r3fs.finger_c_status.position = 0.29
    original_r3fs.finger_c_status.current = 0.07
    
    original_r3fs.scissor_status = Robotiq3FingerActuatorStatus()
    original_r3fs.scissor_status.position_request = 0.4
    original_r3fs.scissor_status.position = 0.39
    original_r3fs.scissor_status.current = 0.08
    
    # Set object statuses
    original_r3fs.finger_a_object_status = Robotiq3FingerObjectStatus()
    original_r3fs.finger_a_object_status.status = Robotiq3FingerObjectStatus.AT_REQUESTED
    original_r3fs.finger_b_object_status = Robotiq3FingerObjectStatus()
    original_r3fs.finger_b_object_status.status = Robotiq3FingerObjectStatus.CONTACT_CLOSING
    original_r3fs.finger_c_object_status = Robotiq3FingerObjectStatus()
    original_r3fs.finger_c_object_status.status = Robotiq3FingerObjectStatus.STOPPED
    original_r3fs.scissor_object_status = Robotiq3FingerObjectStatus()
    original_r3fs.scissor_object_status.status = Robotiq3FingerObjectStatus.IN_MOTION
    
    # Set status values
    original_r3fs.initialization_status = Robotiq3FingerStatus.GRIPPER_ACTIVATION
    original_r3fs.gripper_action_status = Robotiq3FingerStatus.GRIPPER_GOTO
    original_r3fs.gripper_system_status = Robotiq3FingerStatus.GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE
    original_r3fs.gripper_motion_status = Robotiq3FingerStatus.GRIPPER_IN_MOTION
    original_r3fs.gripper_fault_status = Robotiq3FingerStatus.NO_FAULTS
    
    numpy_r3fs = ros2_numpy.numpify(original_r3fs)
    restored_r3fs = ros2_numpy.msgify(Robotiq3FingerStatus, numpy_r3fs)
    
    assert almost_equal(original_r3fs.finger_a_status.position, restored_r3fs.finger_a_status.position)
    assert almost_equal(original_r3fs.finger_b_status.current, restored_r3fs.finger_b_status.current)
    assert original_r3fs.finger_a_object_status.status == restored_r3fs.finger_a_object_status.status
    assert original_r3fs.initialization_status == restored_r3fs.initialization_status
    assert original_r3fs.gripper_fault_status == restored_r3fs.gripper_fault_status
    print("✓ Robotiq3FingerStatus conversion test passed")
    
    print("\n🎉 All conversion tests passed successfully!")
    print(f"Tested {13} message types with full round-trip conversion.")


if __name__ == "__main__":
    _victor_msg_test_conversion()