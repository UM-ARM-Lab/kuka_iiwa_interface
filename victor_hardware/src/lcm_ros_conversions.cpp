#include <utility>
#include <victor_hardware/lcm_ros_conversions.hpp>
#include <arm_utilities/ros_helpers.hpp>
#include <arm_utilities/arm_helpers.hpp>

namespace victor_hardware {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LCM to ROS and ROS to LCM message converters
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

msg::JointValueQuantity jvqLcmToRos(const victor_lcm_interface::joint_value_quantity& lcm_jvq) {
  msg::JointValueQuantity ros_jvq;
  ros_jvq.joint_1 = lcm_jvq.joint_1;
  ros_jvq.joint_2 = lcm_jvq.joint_2;
  ros_jvq.joint_3 = lcm_jvq.joint_3;
  ros_jvq.joint_4 = lcm_jvq.joint_4;
  ros_jvq.joint_5 = lcm_jvq.joint_5;
  ros_jvq.joint_6 = lcm_jvq.joint_6;
  ros_jvq.joint_7 = lcm_jvq.joint_7;
  return ros_jvq;
}

victor_lcm_interface::joint_value_quantity jvqRosToLcm(const msg::JointValueQuantity& ros_jvq) {
  victor_lcm_interface::joint_value_quantity lcm_jvq;
  lcm_jvq.joint_1 = ros_jvq.joint_1;
  lcm_jvq.joint_2 = ros_jvq.joint_2;
  lcm_jvq.joint_3 = ros_jvq.joint_3;
  lcm_jvq.joint_4 = ros_jvq.joint_4;
  lcm_jvq.joint_5 = ros_jvq.joint_5;
  lcm_jvq.joint_6 = ros_jvq.joint_6;
  lcm_jvq.joint_7 = ros_jvq.joint_7;
  return lcm_jvq;
}

msg::CartesianValueQuantity cvqLcmToRos(const victor_lcm_interface::cartesian_value_quantity& lcm_cvq) {
  msg::CartesianValueQuantity ros_cvq;
  ros_cvq.x = lcm_cvq.x;
  ros_cvq.y = lcm_cvq.y;
  ros_cvq.z = lcm_cvq.z;
  ros_cvq.a = lcm_cvq.a;
  ros_cvq.b = lcm_cvq.b;
  ros_cvq.c = lcm_cvq.c;
  return ros_cvq;
}

victor_lcm_interface::cartesian_value_quantity cvqRosToLcm(const msg::CartesianValueQuantity& ros_cvq) {
  victor_lcm_interface::cartesian_value_quantity lcm_cvq;
  lcm_cvq.x = ros_cvq.x;
  lcm_cvq.y = ros_cvq.y;
  lcm_cvq.z = ros_cvq.z;
  lcm_cvq.a = ros_cvq.a;
  lcm_cvq.b = ros_cvq.b;
  lcm_cvq.c = ros_cvq.c;
  return lcm_cvq;
}

geometry_msgs::msg::Pose poseLcmToRos(const victor_lcm_interface::cartesian_pose& lcm_pose) {
  geometry_msgs::msg::Pose ros_pose;
  ros_pose.position.x = lcm_pose.xt;
  ros_pose.position.y = lcm_pose.yt;
  ros_pose.position.z = lcm_pose.zt;
  ros_pose.orientation.w = lcm_pose.wr;
  ros_pose.orientation.x = lcm_pose.xr;
  ros_pose.orientation.y = lcm_pose.yr;
  ros_pose.orientation.z = lcm_pose.zr;
  return ros_pose;
}

victor_lcm_interface::cartesian_pose poseRosToLcm(const geometry_msgs::msg::Pose& ros_pose) {
  victor_lcm_interface::cartesian_pose lcm_pose;
  lcm_pose.xt = ros_pose.position.x;
  lcm_pose.yt = ros_pose.position.y;
  lcm_pose.zt = ros_pose.position.z;
  lcm_pose.wr = ros_pose.orientation.w;
  lcm_pose.xr = ros_pose.orientation.x;
  lcm_pose.yr = ros_pose.orientation.y;
  lcm_pose.zr = ros_pose.orientation.z;
  return lcm_pose;
}

msg::JointImpedanceParameters jointImpedanceParamsLcmToRos(
    const victor_lcm_interface::joint_impedance_parameters& lcm_jip) {
  msg::JointImpedanceParameters ros_jip;
  ros_jip.joint_damping = jvqLcmToRos(lcm_jip.joint_damping);
  ros_jip.joint_stiffness = jvqLcmToRos(lcm_jip.joint_stiffness);
  return ros_jip;
}

victor_lcm_interface::joint_impedance_parameters jointImpedanceParamsRosToLcm(
    const msg::JointImpedanceParameters& ros_jip) {
  victor_lcm_interface::joint_impedance_parameters lcm_jip;
  lcm_jip.joint_damping = jvqRosToLcm(ros_jip.joint_damping);
  lcm_jip.joint_stiffness = jvqRosToLcm(ros_jip.joint_stiffness);
  return lcm_jip;
}

msg::CartesianImpedanceParameters cartesianImpedanceParamsLcmToRos(
    const victor_lcm_interface::cartesian_impedance_parameters& lcm_cip) {
  msg::CartesianImpedanceParameters ros_cip;
  ros_cip.cartesian_damping = cvqLcmToRos(lcm_cip.cartesian_damping);
  ros_cip.nullspace_damping = lcm_cip.nullspace_damping;
  ros_cip.cartesian_stiffness = cvqLcmToRos(lcm_cip.cartesian_stiffness);
  ros_cip.nullspace_stiffness = lcm_cip.nullspace_stiffness;
  return ros_cip;
}

victor_lcm_interface::cartesian_impedance_parameters cartesianImpedanceParamsRosToLcm(
    const msg::CartesianImpedanceParameters& ros_cip) {
  victor_lcm_interface::cartesian_impedance_parameters lcm_cip;
  lcm_cip.cartesian_damping = cvqRosToLcm(ros_cip.cartesian_damping);
  lcm_cip.nullspace_damping = ros_cip.nullspace_damping;
  lcm_cip.cartesian_stiffness = cvqRosToLcm(ros_cip.cartesian_stiffness);
  lcm_cip.nullspace_stiffness = ros_cip.nullspace_stiffness;
  return lcm_cip;
}

msg::JointPathExecutionParameters jointPexpLcmToRos(
    const victor_lcm_interface::joint_path_execution_parameters& lcm_pexp) {
  msg::JointPathExecutionParameters ros_pexp;
  ros_pexp.joint_relative_acceleration = lcm_pexp.joint_relative_acceleration;
  ros_pexp.joint_relative_velocity = lcm_pexp.joint_relative_velocity;
  ros_pexp.override_joint_acceleration = lcm_pexp.override_joint_acceleration;
  return ros_pexp;
}

victor_lcm_interface::joint_path_execution_parameters jointPexpRosToLcm(
    const msg::JointPathExecutionParameters& ros_pexp) {
  victor_lcm_interface::joint_path_execution_parameters lcm_pexp;
  lcm_pexp.joint_relative_acceleration = ros_pexp.joint_relative_acceleration;
  lcm_pexp.joint_relative_velocity = ros_pexp.joint_relative_velocity;
  lcm_pexp.override_joint_acceleration = ros_pexp.override_joint_acceleration;
  return lcm_pexp;
}

msg::CartesianPathExecutionParameters cartesianPexpLcmToRos(
    const victor_lcm_interface::cartesian_path_execution_parameters& lcm_pexp) {
  msg::CartesianPathExecutionParameters ros_pexp;
  ros_pexp.max_velocity = cvqLcmToRos(lcm_pexp.max_velocity);
  ros_pexp.max_acceleration = cvqLcmToRos(lcm_pexp.max_acceleration);
  ros_pexp.max_nullspace_velocity = lcm_pexp.max_nullspace_velocity;
  ros_pexp.max_nullspace_acceleration = lcm_pexp.max_nullspace_acceleration;
  return ros_pexp;
}

victor_lcm_interface::cartesian_path_execution_parameters cartesianPexpRosToLcm(
    const msg::CartesianPathExecutionParameters& ros_pexp) {
  victor_lcm_interface::cartesian_path_execution_parameters lcm_pexp;
  lcm_pexp.max_velocity = cvqRosToLcm(ros_pexp.max_velocity);
  lcm_pexp.max_acceleration = cvqRosToLcm(ros_pexp.max_acceleration);
  lcm_pexp.max_nullspace_velocity = ros_pexp.max_nullspace_velocity;
  lcm_pexp.max_nullspace_acceleration = ros_pexp.max_nullspace_acceleration;
  return lcm_pexp;
}

msg::CartesianControlModeLimits cartesianControlModeLimitsLcmToRos(
    const victor_lcm_interface::cartesian_control_mode_limits& lcm_ccml) {
  msg::CartesianControlModeLimits ros_ccml;
  ros_ccml.max_cartesian_velocity = cvqLcmToRos(lcm_ccml.max_cartesian_velocity);
  ros_ccml.max_path_deviation = cvqLcmToRos(lcm_ccml.max_path_deviation);
  ros_ccml.max_control_force = cvqLcmToRos(lcm_ccml.max_control_force);
  ros_ccml.stop_on_max_control_force = static_cast<unsigned char>(lcm_ccml.stop_on_max_control_force);
  return ros_ccml;
}

victor_lcm_interface::cartesian_control_mode_limits cartesianControlModeLimitsRosToLcm(
    const msg::CartesianControlModeLimits& ros_ccml) {
  victor_lcm_interface::cartesian_control_mode_limits lcm_ccml;
  lcm_ccml.max_cartesian_velocity = cvqRosToLcm(ros_ccml.max_cartesian_velocity);
  lcm_ccml.max_path_deviation = cvqRosToLcm(ros_ccml.max_path_deviation);
  lcm_ccml.max_control_force = cvqRosToLcm(ros_ccml.max_control_force);
  lcm_ccml.stop_on_max_control_force = static_cast<int8_t>(ros_ccml.stop_on_max_control_force);
  return lcm_ccml;
}

msg::ControlMode controlModeLcmToRos(const victor_lcm_interface::control_mode& lcm_cm) {
  msg::ControlMode ros_cm;
  ros_cm.mode = (uint8_t)lcm_cm.mode;
  return ros_cm;
}

victor_lcm_interface::control_mode controlModeRosToLcm(const msg::ControlMode& ros_cm) {
  victor_lcm_interface::control_mode lcm_cm;
  lcm_cm.mode = (int8_t)ros_cm.mode;
  return lcm_cm;
}

msg::MotionStatus motionStatusLcmToRos(const victor_lcm_interface::motion_status& lcm_status) {
  msg::MotionStatus ros_status;
  ros_status.commanded_cartesian_pose = poseLcmToRos(lcm_status.commanded_cartesian_pose);
  ros_status.commanded_cartesian_pose_abc = cvqLcmToRos(lcm_status.commanded_cartesian_pose_abc);
  ros_status.commanded_joint_position = jvqLcmToRos(lcm_status.commanded_joint_position);
  ros_status.estimated_external_torque = jvqLcmToRos(lcm_status.estimated_external_torque);
  ros_status.estimated_external_wrench = cvqLcmToRos(lcm_status.estimated_external_wrench);
  ros_status.measured_cartesian_pose = poseLcmToRos(lcm_status.measured_cartesian_pose);
  ros_status.measured_cartesian_pose_abc = cvqLcmToRos(lcm_status.measured_cartesian_pose_abc);
  ros_status.measured_joint_position = jvqLcmToRos(lcm_status.measured_joint_position);
  ros_status.measured_joint_torque = jvqLcmToRos(lcm_status.measured_joint_torque);
  ros_status.measured_joint_velocity = jvqLcmToRos(lcm_status.measured_joint_velocity);
  ros_status.active_control_mode = controlModeLcmToRos(lcm_status.active_control_mode);
  ros_status.header.stamp = ros_helpers::secondsToTimeMsg(lcm_status.timestamp);
  return ros_status;
}

victor_lcm_interface::motion_command motionCommandRosToLcm(const msg::MotionCommand& ros_command) {
  victor_lcm_interface::motion_command lcm_command{};
  lcm_command.cartesian_pose = poseRosToLcm(ros_command.cartesian_pose);
  lcm_command.joint_position = jvqRosToLcm(ros_command.joint_position);
  lcm_command.joint_velocity = jvqRosToLcm(ros_command.joint_velocity);
  lcm_command.control_mode = controlModeRosToLcm(ros_command.control_mode);
  lcm_command.timestamp = rclcpp::Time(ros_command.header.stamp).seconds();
  return lcm_command;
}

msg::ControlModeParameters controlModeParamsLcmToRos(const victor_lcm_interface::control_mode_parameters& lcm_cmp) {
  msg::ControlModeParameters ros_cmp;
  ros_cmp.cartesian_control_mode_limits = cartesianControlModeLimitsLcmToRos(lcm_cmp.cartesian_control_mode_limits);
  ros_cmp.cartesian_impedance_params = cartesianImpedanceParamsLcmToRos(lcm_cmp.cartesian_impedance_params);
  ros_cmp.joint_impedance_params = jointImpedanceParamsLcmToRos(lcm_cmp.joint_impedance_params);
  ros_cmp.joint_path_execution_params = jointPexpLcmToRos(lcm_cmp.joint_path_execution_params);
  ros_cmp.cartesian_path_execution_params = cartesianPexpLcmToRos(lcm_cmp.cartesian_path_execution_params);
  ros_cmp.control_mode = controlModeLcmToRos(lcm_cmp.control_mode);
  ros_cmp.header.stamp = ros_helpers::secondsToTimeMsg(lcm_cmp.timestamp);
  return ros_cmp;
}

victor_lcm_interface::control_mode_parameters controlModeParamsRosToLcm(const msg::ControlModeParameters& ros_cmp) {
  victor_lcm_interface::control_mode_parameters lcm_cmp{};
  lcm_cmp.cartesian_control_mode_limits = cartesianControlModeLimitsRosToLcm(ros_cmp.cartesian_control_mode_limits);
  lcm_cmp.cartesian_impedance_params = cartesianImpedanceParamsRosToLcm(ros_cmp.cartesian_impedance_params);
  lcm_cmp.joint_impedance_params = jointImpedanceParamsRosToLcm(ros_cmp.joint_impedance_params);
  lcm_cmp.joint_path_execution_params = jointPexpRosToLcm(ros_cmp.joint_path_execution_params);
  lcm_cmp.cartesian_path_execution_params = cartesianPexpRosToLcm(ros_cmp.cartesian_path_execution_params);
  lcm_cmp.control_mode = controlModeRosToLcm(ros_cmp.control_mode);
  lcm_cmp.timestamp = rclcpp::Time(ros_cmp.header.stamp).seconds();
  return lcm_cmp;
}

victor_lcm_interface::robotiq_3finger_actuator_command fingerCommandRosToLcm(
    const msg::Robotiq3FingerActuatorCommand& finger_command) {
  victor_lcm_interface::robotiq_3finger_actuator_command lcm_command{};

  // "Magic" number 0.0 and 1 are the bounds representing percentage value
  lcm_command.position = arm_helpers::ClampValueAndWarn(finger_command.position, 0.0, 1.0);
  lcm_command.speed = arm_helpers::ClampValueAndWarn(finger_command.speed, 0.0, 1.0);
  lcm_command.force = arm_helpers::ClampValueAndWarn(finger_command.force, 0.0, 1.0);
  lcm_command.timestamp = rclcpp::Time(finger_command.header.stamp).seconds();
  return lcm_command;
}

msg::Robotiq3FingerActuatorStatus fingerStatusLcmToRos(
    const victor_lcm_interface::robotiq_3finger_actuator_status& finger_status) {
  msg::Robotiq3FingerActuatorStatus ros_status;
  ros_status.position = finger_status.position;
  ros_status.position_request = finger_status.position_request;
  ros_status.current = finger_status.current;
  ros_status.header.stamp = ros_helpers::secondsToTimeMsg(finger_status.timestamp);
  return ros_status;
}

rclcpp::Time secondsToStamp(const victor_lcm_interface::robotiq_3finger_object_status& object_status);
msg::Robotiq3FingerObjectStatus objectStatusLcmToRos(
    const victor_lcm_interface::robotiq_3finger_object_status& object_status) {
  msg::Robotiq3FingerObjectStatus ros_status;
  ros_status.status = (uint8_t)object_status.status;
  ros_status.header.stamp = ros_helpers::secondsToTimeMsg(object_status.timestamp);
  return ros_status;
}

msg::Robotiq3FingerStatus gripperStatusLcmToRos(const victor_lcm_interface::robotiq_3finger_status& status) {
  msg::Robotiq3FingerStatus ros_status;
  ros_status.finger_a_status = fingerStatusLcmToRos(status.finger_a_status);
  ros_status.finger_b_status = fingerStatusLcmToRos(status.finger_b_status);
  ros_status.finger_c_status = fingerStatusLcmToRos(status.finger_c_status);
  ros_status.scissor_status = fingerStatusLcmToRos(status.scissor_status);
  ros_status.finger_a_object_status = objectStatusLcmToRos(status.finger_a_object_status);
  ros_status.finger_b_object_status = objectStatusLcmToRos(status.finger_b_object_status);
  ros_status.finger_c_object_status = objectStatusLcmToRos(status.finger_c_object_status);
  ros_status.scissor_object_status = objectStatusLcmToRos(status.scissor_object_status);
  ros_status.gripper_action_status = (uint8_t)status.gripper_action_status;
  ros_status.gripper_system_status = (uint8_t)status.gripper_system_status;
  ros_status.gripper_motion_status = (uint8_t)status.gripper_motion_status;
  ros_status.gripper_fault_status = (uint8_t)status.gripper_fault_status;
  ros_status.initialization_status = (uint8_t)status.initialization_status;
  ros_status.header.stamp = ros_helpers::secondsToTimeMsg(status.timestamp);
  return ros_status;
}

victor_lcm_interface::robotiq_3finger_command gripperCommandRosToLcm(const msg::Robotiq3FingerCommand& command) {
  victor_lcm_interface::robotiq_3finger_command lcm_command{};
  lcm_command.finger_a_command = fingerCommandRosToLcm(command.finger_a_command);
  lcm_command.finger_b_command = fingerCommandRosToLcm(command.finger_b_command);
  lcm_command.finger_c_command = fingerCommandRosToLcm(command.finger_c_command);
  lcm_command.scissor_command = fingerCommandRosToLcm(command.scissor_command);
  lcm_command.timestamp = rclcpp::Time(command.header.stamp.sec).seconds();
  return lcm_command;
}

}  // namespace victor_hardware